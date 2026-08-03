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
  const std::regex pattern("\"" + _key + "\"\\s*:\\s*([-+0-9.eE]+)");
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

gz::math::Vector3d Limited(const gz::math::Vector3d &_value, double _limit)
{
  const double length = _value.Length();
  if (_limit <= 0.0 || length <= _limit || length <= 1e-9)
    return _value;
  return _value * (_limit / length);
}
}

class AerodynamicEnvironment:
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
    this->link = gz::sim::Link(this->model.LinkByName(
        _ecm, SdfString(_sdf, "link_name", "base_link")));
    this->upperJoint = gz::sim::Joint(this->model.JointByName(
        _ecm, SdfString(_sdf, "upper_joint_name", "upper_rotor_joint")));
    this->lowerJoint = gz::sim::Joint(this->model.JointByName(
        _ecm, SdfString(_sdf, "lower_joint_name", "lower_rotor_joint")));

    this->enabled = SdfBool(_sdf, "enabled", false);
    this->airDensity = SdfDouble(_sdf, "air_density_kg_m3", this->airDensity);
    this->dragArea.X(SdfDouble(_sdf, "drag_area_x_m2", this->dragArea.X()));
    this->dragArea.Y(SdfDouble(_sdf, "drag_area_y_m2", this->dragArea.Y()));
    this->dragArea.Z(SdfDouble(_sdf, "drag_area_z_m2", this->dragArea.Z()));
    this->angularLinear.X(SdfDouble(
        _sdf, "angular_damping_roll_nm_s", this->angularLinear.X()));
    this->angularLinear.Y(SdfDouble(
        _sdf, "angular_damping_pitch_nm_s", this->angularLinear.Y()));
    this->angularLinear.Z(SdfDouble(
        _sdf, "angular_damping_yaw_nm_s", this->angularLinear.Z()));
    this->angularQuadratic.X(SdfDouble(
        _sdf, "angular_damping_roll_quadratic", this->angularQuadratic.X()));
    this->angularQuadratic.Y(SdfDouble(
        _sdf, "angular_damping_pitch_quadratic", this->angularQuadratic.Y()));
    this->angularQuadratic.Z(SdfDouble(
        _sdf, "angular_damping_yaw_quadratic", this->angularQuadratic.Z()));
    this->nominalMass = SdfDouble(_sdf, "nominal_mass_kg", this->nominalMass);
    this->nominalInertia.X(SdfDouble(_sdf, "nominal_ixx", this->nominalInertia.X()));
    this->nominalInertia.Y(SdfDouble(_sdf, "nominal_iyy", this->nominalInertia.Y()));
    this->nominalInertia.Z(SdfDouble(_sdf, "nominal_izz", this->nominalInertia.Z()));
    this->thrustCoeff = SdfDouble(_sdf, "thrust_coeff", this->thrustCoeff);
    this->parameters.enabled = this->enabled;
    this->parameters.airDensity = this->airDensity;
    this->parameters.dragArea = this->dragArea;
    this->parameters.angularLinear = this->angularLinear;
    this->parameters.angularQuadratic = this->angularQuadratic;
    this->parameters.nominalMass = this->nominalMass;
    this->parameters.nominalInertia = this->nominalInertia;

    this->link.EnableVelocityChecks(_ecm, true);
    this->upperJoint.EnableVelocityCheck(_ecm, true);
    this->lowerJoint.EnableVelocityCheck(_ecm, true);
    this->node.Subscribe("/coaxial_uav/control/config",
        &AerodynamicEnvironment::OnConfig, this);
    this->node.Subscribe("/coaxial_uav/aerodynamics/config",
        &AerodynamicEnvironment::OnConfig, this);
    this->statusPub = this->node.Advertise<gz::msgs::StringMsg>(
        "/coaxial_uav/aerodynamics/status");
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0 || !this->link.Valid(_ecm))
      return;

    const auto pose = this->link.WorldPose(_ecm);
    const auto linearWorld = this->link.WorldLinearVelocity(_ecm);
    const auto angularWorld = this->link.WorldAngularVelocity(_ecm);
    if (!pose || !linearWorld || !angularWorld)
      return;

    Parameters cfg;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      cfg = this->parameters;
    }

    this->UpdateGust(cfg, dt);
    const gz::math::Vector3d windWorld = cfg.windWorld + this->gustWorld;
    const gz::math::Vector3d relativeBody =
        pose->Rot().RotateVectorReverse(*linearWorld - windWorld);
    const gz::math::Vector3d angularBody =
        pose->Rot().RotateVectorReverse(*angularWorld);

    gz::math::Vector3d dragBody;
    dragBody.X(-0.5 * cfg.airDensity * cfg.dragArea.X() *
        std::abs(relativeBody.X()) * relativeBody.X());
    dragBody.Y(-0.5 * cfg.airDensity * cfg.dragArea.Y() *
        std::abs(relativeBody.Y()) * relativeBody.Y());
    dragBody.Z(-0.5 * cfg.airDensity * cfg.dragArea.Z() *
        std::abs(relativeBody.Z()) * relativeBody.Z());

    gz::math::Vector3d dampingBody;
    for (int axis = 0; axis < 3; ++axis)
    {
      dampingBody[axis] = -cfg.angularLinear[axis] * angularBody[axis] -
          cfg.angularQuadratic[axis] * std::abs(angularBody[axis]) *
          angularBody[axis];
    }

    gz::math::Vector3d massForceWorld;
    gz::math::Vector3d inertiaTorqueBody;
    if (this->velocityInitialized)
    {
      const auto linearAcceleration =
          (*linearWorld - this->previousLinearWorld) / dt;
      const auto angularAcceleration =
          (angularBody - this->previousAngularBody) / dt;
      const double filter = 1.0 - std::exp(-dt / 0.05);
      this->filteredLinearAcceleration +=
          filter * (linearAcceleration - this->filteredLinearAcceleration);
      this->filteredAngularAcceleration +=
          filter * (angularAcceleration - this->filteredAngularAcceleration);
      const gz::math::Vector3d gravityWorld(0.0, 0.0, -9.8);
      massForceWorld = -(cfg.massScale - 1.0) * cfg.nominalMass *
          (this->filteredLinearAcceleration - gravityWorld);
      for (int axis = 0; axis < 3; ++axis)
      {
        inertiaTorqueBody[axis] =
            -(cfg.inertiaScale[axis] - 1.0) *
            cfg.nominalInertia[axis] * this->filteredAngularAcceleration[axis];
      }
      massForceWorld = Limited(massForceWorld, 45.0);
      inertiaTorqueBody = Limited(inertiaTorqueBody, 1.5);
    }
    this->velocityInitialized = true;
    this->previousLinearWorld = *linearWorld;
    this->previousAngularBody = angularBody;

    const auto upperVelocity = this->upperJoint.Velocity(_ecm);
    const auto lowerVelocity = this->lowerJoint.Velocity(_ecm);
    const double upperOmega = upperVelocity && !upperVelocity->empty() ?
        std::abs(upperVelocity->front()) : 0.0;
    const double lowerOmega = lowerVelocity && !lowerVelocity->empty() ?
        std::abs(lowerVelocity->front()) : 0.0;
    const double estimatedThrust = this->thrustCoeff *
        (upperOmega * upperOmega + lowerOmega * lowerOmega);
    const gz::math::Vector3d thrustBody(0.0, 0.0, estimatedThrust);
    const gz::math::Vector3d cgTorqueBody = (-cfg.cgOffset).Cross(thrustBody);

    gz::math::Vector3d forceWorld;
    gz::math::Vector3d torqueBody;
    if (cfg.enabled)
    {
      forceWorld = pose->Rot().RotateVector(dragBody) + massForceWorld;
      torqueBody = dampingBody + inertiaTorqueBody + cgTorqueBody;
      this->link.AddWorldWrench(
          _ecm, forceWorld, pose->Rot().RotateVector(torqueBody));
    }
    else
    {
      this->gustWorld.Set(0.0, 0.0, 0.0);
      this->filteredLinearAcceleration.Set(0.0, 0.0, 0.0);
      this->filteredAngularAcceleration.Set(0.0, 0.0, 0.0);
    }

    this->PublishStatus(_info, cfg, windWorld, relativeBody, forceWorld,
        torqueBody, dragBody, dampingBody, massForceWorld, inertiaTorqueBody,
        cgTorqueBody);
  }

  private: struct Parameters
  {
    bool enabled{false};
    double airDensity{1.225};
    gz::math::Vector3d dragArea{0.12, 0.18, 0.10};
    gz::math::Vector3d angularLinear{0.12, 0.18, 0.10};
    gz::math::Vector3d angularQuadratic{0.03, 0.04, 0.02};
    gz::math::Vector3d windWorld{0.0, 0.0, 0.0};
    double gustRms{0.40};
    double gustCorrelationTime{0.80};
    double massScale{1.03};
    gz::math::Vector3d inertiaScale{1.05, 0.97, 1.04};
    gz::math::Vector3d cgOffset{0.010, -0.008, 0.005};
    double nominalMass{8.2};
    gz::math::Vector3d nominalInertia{0.19588, 0.35588, 0.30};
  };

  private: void UpdateGust(const Parameters &_cfg, double _dt)
  {
    if (!_cfg.enabled || _cfg.gustRms <= 0.0)
    {
      this->gustWorld.Set(0.0, 0.0, 0.0);
      return;
    }
    const double tau = std::max(0.05, _cfg.gustCorrelationTime);
    const double decay = std::exp(-_dt / tau);
    const double innovation =
        _cfg.gustRms * std::sqrt(std::max(0.0, 1.0 - decay * decay));
    for (int axis = 0; axis < 3; ++axis)
      this->gustWorld[axis] =
          decay * this->gustWorld[axis] + innovation * this->normal(this->rng);
  }

  private: void OnConfig(const gz::msgs::StringMsg &_msg)
  {
    const auto text = _msg.data();
    std::lock_guard<std::mutex> lock(this->mutex);
    auto &cfg = this->parameters;
    cfg.enabled = BoolField(text, "aerodynamics_enabled", cfg.enabled);
    cfg.airDensity = Clamp(NumberField(
        text, "air_density_kg_m3", cfg.airDensity), 0.5, 2.0);
    cfg.dragArea.X(Clamp(NumberField(
        text, "drag_area_x_m2", cfg.dragArea.X()), 0.0, 2.0));
    cfg.dragArea.Y(Clamp(NumberField(
        text, "drag_area_y_m2", cfg.dragArea.Y()), 0.0, 2.0));
    cfg.dragArea.Z(Clamp(NumberField(
        text, "drag_area_z_m2", cfg.dragArea.Z()), 0.0, 2.0));
    cfg.angularLinear.X(Clamp(NumberField(
        text, "angular_damping_roll_nm_s", cfg.angularLinear.X()), 0.0, 10.0));
    cfg.angularLinear.Y(Clamp(NumberField(
        text, "angular_damping_pitch_nm_s", cfg.angularLinear.Y()), 0.0, 10.0));
    cfg.angularLinear.Z(Clamp(NumberField(
        text, "angular_damping_yaw_nm_s", cfg.angularLinear.Z()), 0.0, 10.0));
    cfg.windWorld.X(Clamp(NumberField(
        text, "wind_x_m_s", cfg.windWorld.X()), -30.0, 30.0));
    cfg.windWorld.Y(Clamp(NumberField(
        text, "wind_y_m_s", cfg.windWorld.Y()), -30.0, 30.0));
    cfg.windWorld.Z(Clamp(NumberField(
        text, "wind_z_m_s", cfg.windWorld.Z()), -15.0, 15.0));
    cfg.gustRms = Clamp(NumberField(text, "gust_rms_m_s", cfg.gustRms), 0.0, 10.0);
    cfg.gustCorrelationTime = Clamp(NumberField(
        text, "gust_correlation_time_s", cfg.gustCorrelationTime), 0.05, 20.0);
    cfg.massScale = Clamp(NumberField(text, "mass_scale", cfg.massScale), 0.7, 1.3);
    cfg.inertiaScale.X(Clamp(NumberField(
        text, "inertia_scale_roll", cfg.inertiaScale.X()), 0.5, 1.5));
    cfg.inertiaScale.Y(Clamp(NumberField(
        text, "inertia_scale_pitch", cfg.inertiaScale.Y()), 0.5, 1.5));
    cfg.inertiaScale.Z(Clamp(NumberField(
        text, "inertia_scale_yaw", cfg.inertiaScale.Z()), 0.5, 1.5));
    cfg.cgOffset.X(Clamp(NumberField(
        text, "cg_offset_x_m", cfg.cgOffset.X()), -0.15, 0.15));
    cfg.cgOffset.Y(Clamp(NumberField(
        text, "cg_offset_y_m", cfg.cgOffset.Y()), -0.15, 0.15));
    cfg.cgOffset.Z(Clamp(NumberField(
        text, "cg_offset_z_m", cfg.cgOffset.Z()), -0.15, 0.15));
    const auto requestedSeed = static_cast<std::uint32_t>(Clamp(NumberField(
        text, "aerodynamics_seed", static_cast<double>(this->seed)),
        0.0, 4294967295.0));
    if (requestedSeed != this->seed)
    {
      this->seed = requestedSeed;
      this->rng.seed(this->seed);
      this->gustWorld.Set(0.0, 0.0, 0.0);
    }
  }

  private: void PublishStatus(const gz::sim::UpdateInfo &_info,
      const Parameters &_cfg, const gz::math::Vector3d &_windWorld,
      const gz::math::Vector3d &_relativeBody,
      const gz::math::Vector3d &_forceWorld,
      const gz::math::Vector3d &_torqueBody,
      const gz::math::Vector3d &_dragBody,
      const gz::math::Vector3d &_dampingBody,
      const gz::math::Vector3d &_massForceWorld,
      const gz::math::Vector3d &_inertiaTorqueBody,
      const gz::math::Vector3d &_cgTorqueBody)
  {
    const double simTime = std::chrono::duration<double>(_info.simTime).count();
    if (simTime - this->lastStatusTime < 0.02)
      return;
    this->lastStatusTime = simTime;
    std::ostringstream out;
    out << "{\"enabled\":" << (_cfg.enabled ? "true" : "false")
        << ",\"wind_world_m_s\":[" << _windWorld.X() << ","
        << _windWorld.Y() << "," << _windWorld.Z() << "]"
        << ",\"relative_air_body_m_s\":[" << _relativeBody.X() << ","
        << _relativeBody.Y() << "," << _relativeBody.Z() << "]"
        << ",\"drag_body_n\":[" << _dragBody.X() << ","
        << _dragBody.Y() << "," << _dragBody.Z() << "]"
        << ",\"damping_body_nm\":[" << _dampingBody.X() << ","
        << _dampingBody.Y() << "," << _dampingBody.Z() << "]"
        << ",\"mass_force_world_n\":[" << _massForceWorld.X() << ","
        << _massForceWorld.Y() << "," << _massForceWorld.Z() << "]"
        << ",\"inertia_torque_body_nm\":[" << _inertiaTorqueBody.X() << ","
        << _inertiaTorqueBody.Y() << "," << _inertiaTorqueBody.Z() << "]"
        << ",\"cg_torque_body_nm\":[" << _cgTorqueBody.X() << ","
        << _cgTorqueBody.Y() << "," << _cgTorqueBody.Z() << "]"
        << ",\"force_world_n\":[" << _forceWorld.X() << ","
        << _forceWorld.Y() << "," << _forceWorld.Z() << "]"
        << ",\"torque_body_nm\":[" << _torqueBody.X() << ","
        << _torqueBody.Y() << "," << _torqueBody.Z() << "]"
        << ",\"mass_scale\":" << _cfg.massScale
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
  private: Parameters parameters;
  private: bool enabled{false};
  private: double airDensity{1.225};
  private: gz::math::Vector3d dragArea{0.12, 0.18, 0.10};
  private: gz::math::Vector3d angularLinear{0.12, 0.18, 0.10};
  private: gz::math::Vector3d angularQuadratic{0.03, 0.04, 0.02};
  private: double nominalMass{8.2};
  private: gz::math::Vector3d nominalInertia{0.19588, 0.35588, 0.30};
  private: double thrustCoeff{2.1610671e-3};
  private: bool velocityInitialized{false};
  private: gz::math::Vector3d previousLinearWorld;
  private: gz::math::Vector3d previousAngularBody;
  private: gz::math::Vector3d filteredLinearAcceleration;
  private: gz::math::Vector3d filteredAngularAcceleration;
  private: gz::math::Vector3d gustWorld;
  private: std::uint32_t seed{20260727u};
  private: std::mt19937 rng{this->seed};
  private: std::normal_distribution<double> normal{0.0, 1.0};
  private: double lastStatusTime{-1.0};
};
}

GZ_ADD_PLUGIN(coaxial_uav::AerodynamicEnvironment,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(coaxial_uav::AerodynamicEnvironment,
    "coaxial_uav::AerodynamicEnvironment")
