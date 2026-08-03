#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <random>
#include <regex>
#include <sstream>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace coaxial_uav
{
namespace
{
double Clamp(double _value, double _low, double _high)
{
  return std::max(_low, std::min(_high, _value));
}

double NumberField(const std::string &_text, const std::string &_key,
                   double _fallback)
{
  const std::regex pattern("\"" + _key +
      "\"\\s*:\\s*([-+0-9.eE]+)");
  std::smatch match;
  if (std::regex_search(_text, match, pattern) && match.size() > 1)
    return std::stod(match[1].str());
  return _fallback;
}

bool BoolField(const std::string &_text, const std::string &_key,
               bool _fallback)
{
  const std::regex pattern("\"" + _key + "\"\\s*:\\s*(true|false|1|0)");
  std::smatch match;
  if (!std::regex_search(_text, match, pattern) || match.size() <= 1)
    return _fallback;
  const auto value = match[1].str();
  return value == "true" || value == "1";
}

std::string StringField(const std::string &_text, const std::string &_key,
                        const std::string &_fallback)
{
  const std::regex pattern("\"" + _key + "\"\\s*:\\s*\"([^\"]+)\"");
  std::smatch match;
  if (std::regex_search(_text, match, pattern) && match.size() > 1)
    return match[1].str();
  return _fallback;
}

double SdfDouble(const std::shared_ptr<const sdf::Element> &_sdf,
                 const std::string &_key, double _fallback)
{
  if (_sdf && _sdf->HasElement(_key))
    return _sdf->Get<double>(_key);
  return _fallback;
}

bool SdfBool(const std::shared_ptr<const sdf::Element> &_sdf,
             const std::string &_key, bool _fallback)
{
  if (_sdf && _sdf->HasElement(_key))
    return _sdf->Get<bool>(_key);
  return _fallback;
}

std::string SdfString(const std::shared_ptr<const sdf::Element> &_sdf,
                      const std::string &_key,
                      const std::string &_fallback)
{
  if (_sdf && _sdf->HasElement(_key))
    return _sdf->Get<std::string>(_key);
  return _fallback;
}

struct DisturbanceParameters
{
  double liftGain{0.0};
  double verticalDamping{0.0};
  double rollBias{0.0};
  double pitchBias{0.0};
  double yawBias{0.0};
  double rollPitchRms{0.0};
  double yawRms{0.0};
  double correlationTime{0.25};
};

DisturbanceParameters Preset(const std::string &_name)
{
  if (_name == "calm")
    return {0.03, 8.0, 0.0, 0.0, 0.0, 0.020, 0.005, 0.35};
  if (_name == "mild")
    return {0.07, 16.0, 0.015, -0.010, 0.0, 0.060, 0.015, 0.25};
  if (_name == "strong")
    return {0.12, 28.0, 0.030, -0.020, 0.0, 0.120, 0.030, 0.18};
  if (_name == "asymmetric")
    return {0.08, 18.0, 0.100, 0.050, 0.010, 0.060, 0.015, 0.30};
  return {};
}
}

class NearSurfaceDisturbance:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &/*_eventMgr*/) override
  {
    this->model = gz::sim::Model(_entity);
    const auto linkName = SdfString(_sdf, "link_name", "base_link");
    const auto upperJointName = SdfString(_sdf, "upper_joint_name",
        "upper_rotor_joint");
    const auto lowerJointName = SdfString(_sdf, "lower_joint_name",
        "lower_rotor_joint");
    this->link = gz::sim::Link(this->model.LinkByName(_ecm, linkName));
    this->upperJoint = gz::sim::Joint(
        this->model.JointByName(_ecm, upperJointName));
    this->lowerJoint = gz::sim::Joint(
        this->model.JointByName(_ecm, lowerJointName));

    this->enabled = SdfBool(_sdf, "enabled", false);
    this->preset = SdfString(_sdf, "preset", "off");
    this->waterZ = SdfDouble(_sdf, "water_z_m", this->waterZ);
    this->rotorRadius = SdfDouble(_sdf, "rotor_radius_m", this->rotorRadius);
    this->rotorPlaneOffset =
        SdfDouble(_sdf, "rotor_plane_offset_m", this->rotorPlaneOffset);
    this->thrustCoeff =
        SdfDouble(_sdf, "thrust_coeff", this->thrustCoeff);
    this->fullEffectRatio =
        SdfDouble(_sdf, "full_effect_height_ratio", this->fullEffectRatio);
    this->zeroEffectRatio =
        SdfDouble(_sdf, "zero_effect_height_ratio", this->zeroEffectRatio);
    this->minimumRotorOmega =
        SdfDouble(_sdf, "minimum_rotor_omega_rad_s", this->minimumRotorOmega);
    this->parameters = Preset(this->preset);

    this->link.EnableVelocityChecks(_ecm, true);
    this->upperJoint.EnableVelocityCheck(_ecm, true);
    this->lowerJoint.EnableVelocityCheck(_ecm, true);
    this->node.Subscribe("/coaxial_uav/disturbance/config",
        &NearSurfaceDisturbance::OnConfig, this);
    this->node.Subscribe("/coaxial_uav/control/config",
        &NearSurfaceDisturbance::OnConfig, this);
    this->statusPub = this->node.Advertise<gz::msgs::StringMsg>(
        "/coaxial_uav/disturbance/status");
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0 || !this->link.Valid(_ecm))
      return;

    bool enabled;
    std::string preset;
    DisturbanceParameters parameters;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      enabled = this->enabled;
      preset = this->preset;
      parameters = this->parameters;
    }

    const auto pose = this->link.WorldPose(_ecm);
    const auto linearVel = this->link.WorldLinearVelocity(_ecm);
    if (!pose || !linearVel)
      return;
    const auto upperVelocity = this->upperJoint.Velocity(_ecm);
    const auto lowerVelocity = this->lowerJoint.Velocity(_ecm);
    const double upperOmega = upperVelocity && !upperVelocity->empty() ?
        std::abs(upperVelocity->front()) : 0.0;
    const double lowerOmega = lowerVelocity && !lowerVelocity->empty() ?
        std::abs(lowerVelocity->front()) : 0.0;
    const double meanOmega = 0.5 * (upperOmega + lowerOmega);

    const double diskHeight =
        pose->Z() + this->rotorPlaneOffset - this->waterZ;
    const double heightRatio = diskHeight / std::max(this->rotorRadius, 1e-6);
    const double range =
        std::max(this->zeroEffectRatio - this->fullEffectRatio, 1e-6);
    const double u = Clamp(
        (this->zeroEffectRatio - heightRatio) / range, 0.0, 1.0);
    double envelope = u * u * (3.0 - 2.0 * u);
    const bool active = enabled && preset != "off" &&
        meanOmega >= this->minimumRotorOmega;
    if (!active)
      envelope = 0.0;

    this->UpdateNoise(parameters, dt, active);
    const double estimatedThrust =
        2.0 * this->thrustCoeff * meanOmega * meanOmega;
    const double rawForceZ = parameters.liftGain * estimatedThrust -
        parameters.verticalDamping * linearVel->Z();
    const double forceZ = envelope * Clamp(rawForceZ, -16.0, 16.0);
    const gz::math::Vector3d bodyTorque(
        envelope * (parameters.rollBias + this->rollNoise),
        envelope * (parameters.pitchBias + this->pitchNoise),
        envelope * (parameters.yawBias + this->yawNoise));
    const auto worldTorque = pose->Rot().RotateVector(bodyTorque);
    if (active && envelope > 0.0)
    {
      this->link.AddWorldWrench(
          _ecm, gz::math::Vector3d(0.0, 0.0, forceZ), worldTorque);
    }
    this->PublishStatus(_info, preset, enabled, active, diskHeight,
        heightRatio, envelope, meanOmega, estimatedThrust, forceZ, bodyTorque);
  }

  private: void UpdateNoise(const DisturbanceParameters &_parameters,
                            double _dt, bool _active)
  {
    if (!_active)
    {
      this->rollNoise = 0.0;
      this->pitchNoise = 0.0;
      this->yawNoise = 0.0;
      return;
    }
    const double tau = std::max(_parameters.correlationTime, 0.01);
    const double decay = std::exp(-_dt / tau);
    const double innovation = std::sqrt(std::max(0.0, 1.0 - decay * decay));
    this->rollNoise = decay * this->rollNoise +
        _parameters.rollPitchRms * innovation * this->normal(this->rng);
    this->pitchNoise = decay * this->pitchNoise +
        _parameters.rollPitchRms * innovation * this->normal(this->rng);
    this->yawNoise = decay * this->yawNoise +
        _parameters.yawRms * innovation * this->normal(this->rng);
  }

  private: void OnConfig(const gz::msgs::StringMsg &_msg)
  {
    const auto text = _msg.data();
    std::lock_guard<std::mutex> lock(this->mutex);
    this->enabled = BoolField(text, "disturbance_enabled",
        BoolField(text, "enabled", this->enabled));
    const auto requestedPreset = StringField(text, "disturbance_preset",
        StringField(text, "preset", this->preset));
    if (requestedPreset == "off" || requestedPreset == "calm" ||
        requestedPreset == "mild" || requestedPreset == "strong" ||
        requestedPreset == "asymmetric")
    {
      this->preset = requestedPreset;
      this->parameters = Preset(this->preset);
    }
    const auto requestedSeed = static_cast<std::uint32_t>(Clamp(
        NumberField(text, "disturbance_seed",
            NumberField(text, "seed", static_cast<double>(this->seed))),
        0.0, 4294967295.0));
    if (requestedSeed != this->seed)
    {
      this->seed = requestedSeed;
      this->rng.seed(this->seed);
      this->rollNoise = 0.0;
      this->pitchNoise = 0.0;
      this->yawNoise = 0.0;
    }
  }

  private: void PublishStatus(const gz::sim::UpdateInfo &_info,
      const std::string &_preset, bool _enabled, bool _active,
      double _diskHeight, double _heightRatio, double _envelope,
      double _meanOmega, double _estimatedThrust, double _forceZ,
      const gz::math::Vector3d &_bodyTorque)
  {
    const double simTime = std::chrono::duration<double>(_info.simTime).count();
    if (simTime - this->lastStatusTime < 0.003)
      return;
    this->lastStatusTime = simTime;

    std::ostringstream out;
    out << "{\"enabled\":" << (_enabled ? "true" : "false")
        << ",\"active\":" << (_active ? "true" : "false")
        << ",\"preset\":\"" << _preset << "\""
        << ",\"disk_height_m\":" << _diskHeight
        << ",\"height_ratio\":" << _heightRatio
        << ",\"envelope\":" << _envelope
        << ",\"mean_rotor_omega_rad_s\":" << _meanOmega
        << ",\"estimated_thrust_n\":" << _estimatedThrust
        << ",\"force_z_n\":" << _forceZ
        << ",\"roll_torque_nm\":" << _bodyTorque.X()
        << ",\"pitch_torque_nm\":" << _bodyTorque.Y()
        << ",\"yaw_torque_nm\":" << _bodyTorque.Z()
        << ",\"sim_time_s\":" << simTime << "}";
    gz::msgs::StringMsg msg;
    msg.set_data(out.str());
    this->statusPub.Publish(msg);
  }

  private: gz::sim::Model model;
  private: gz::sim::Link link;
  private: gz::sim::Joint upperJoint;
  private: gz::sim::Joint lowerJoint;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher statusPub;
  private: std::mutex mutex;

  private: bool enabled{false};
  private: std::string preset{"off"};
  private: DisturbanceParameters parameters;
  private: double waterZ{0.0};
  private: double rotorRadius{0.775};
  private: double rotorPlaneOffset{0.235};
  private: double thrustCoeff{2.1610671e-3};
  private: double fullEffectRatio{0.30};
  private: double zeroEffectRatio{1.50};
  private: double minimumRotorOmega{20.0};
  private: std::uint32_t seed{20260726u};
  private: std::mt19937 rng{this->seed};
  private: std::normal_distribution<double> normal{0.0, 1.0};
  private: double rollNoise{0.0};
  private: double pitchNoise{0.0};
  private: double yawNoise{0.0};
  private: double lastStatusTime{-1.0};
};
}

GZ_ADD_PLUGIN(coaxial_uav::NearSurfaceDisturbance,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(coaxial_uav::NearSurfaceDisturbance,
    "coaxial_uav::NearSurfaceDisturbance")
