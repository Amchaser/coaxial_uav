#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>

#include <gz/math/Pose3.hh>
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
constexpr double kPi = 3.14159265358979323846;

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

std::string StringField(const std::string &_text, const std::string &_key,
                        const std::string &_fallback)
{
  const std::regex pattern("\"" + _key + "\"\\s*:\\s*\"([^\"]*)\"");
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

double CircularSegmentArea(double _radius, double _depth)
{
  const double h = Clamp(_depth, 0.0, 2.0 * _radius);
  if (h <= 0.0)
    return 0.0;
  if (h >= 2.0 * _radius)
    return kPi * _radius * _radius;
  const double y = _radius - h;
  return _radius * _radius * std::acos(y / _radius) -
      y * std::sqrt(std::max(0.0, 2.0 * _radius * h - h * h));
}

gz::math::Vector3d Limited(const gz::math::Vector3d &_value, double _limit)
{
  const double length = _value.Length();
  if (_limit <= 0.0 || length <= _limit || length <= 1e-9)
    return _value;
  return _value * (_limit / length);
}
}

class CoaxialWaterInteraction:
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

    this->parameters.rotorInterferenceEnabled =
        SdfBool(_sdf, "rotor_interference_enabled", true);
    this->parameters.maxThrustLoss = Clamp(SdfDouble(
        _sdf, "coaxial_max_thrust_loss", 0.06), 0.0, 0.35);
    this->parameters.inflowTimeConstant = Clamp(SdfDouble(
        _sdf, "coaxial_inflow_time_constant_s", 0.12), 0.01, 2.0);
    this->parameters.minimumRotorOmega = std::max(0.0, SdfDouble(
        _sdf, "minimum_rotor_omega_rad_s", 20.0));
    this->parameters.thrustCoeff =
        SdfDouble(_sdf, "thrust_coeff", 2.1610671e-3);
    this->parameters.momentConstant =
        SdfDouble(_sdf, "moment_constant", 0.06478);

    this->parameters.hydrodynamicsEnabled =
        SdfBool(_sdf, "hydrodynamics_enabled", true);
    this->parameters.vehicleMass = Clamp(SdfDouble(
        _sdf, "vehicle_mass_kg", 8.2), 0.1, 1000.0);
    this->parameters.waterDensity = Clamp(SdfDouble(
        _sdf, "water_density_kg_m3", 997.0), 500.0, 1300.0);
    this->parameters.waterLevel = SdfDouble(_sdf, "water_level_z_m", 0.0);
    this->parameters.virtualDraft = Clamp(SdfDouble(
        _sdf, "float_virtual_draft_m", 0.055), 0.0, 0.18);
    this->parameters.floatRadius = Clamp(SdfDouble(
        _sdf, "float_radius_m", 0.09), 0.01, 0.5);
    this->parameters.floatLength = Clamp(SdfDouble(
        _sdf, "float_length_m", 1.10), 0.05, 5.0);
    this->parameters.floatLateralOffset = SdfDouble(
        _sdf, "float_lateral_offset_m", 0.34);
    this->parameters.floatVerticalOffset = SdfDouble(
        _sdf, "float_vertical_offset_m", -0.24);
    this->parameters.floatCount = Clamp(SdfDouble(
        _sdf, "float_count", 2.0), 1.0, 16.0);
    this->parameters.linearDrag = gz::math::Vector3d(
        SdfDouble(_sdf, "water_linear_drag_x_n_s_m", 4.0),
        SdfDouble(_sdf, "water_linear_drag_y_n_s_m", 35.0),
        SdfDouble(_sdf, "water_linear_drag_z_n_s_m", 80.0));
    this->parameters.quadraticDrag = gz::math::Vector3d(
        SdfDouble(_sdf, "water_quadratic_drag_x", 0.25),
        SdfDouble(_sdf, "water_quadratic_drag_y", 1.00),
        SdfDouble(_sdf, "water_quadratic_drag_z", 1.10));
    this->parameters.waterCurrent = gz::math::Vector3d(
        SdfDouble(_sdf, "water_current_x_m_s", 0.0),
        SdfDouble(_sdf, "water_current_y_m_s", 0.0),
        SdfDouble(_sdf, "water_current_z_m_s", 0.0));
    this->parameters.movingTargetEnabled = false;
    this->parameters.slammingGain = Clamp(SdfDouble(
        _sdf, "water_slamming_gain_n_s_m", 35.0), 0.0, 500.0);
    this->parameters.maxHydrodynamicForce = Clamp(SdfDouble(
        _sdf, "max_hydrodynamic_force_n", 220.0), 10.0, 2000.0);

    this->link.EnableVelocityChecks(_ecm, true);
    this->upperJoint.EnableVelocityCheck(_ecm, true);
    this->lowerJoint.EnableVelocityCheck(_ecm, true);
    this->node.Subscribe("/coaxial_uav/control/config",
        &CoaxialWaterInteraction::OnConfig, this);
    this->node.Subscribe("/coaxial_uav/control/status",
        &CoaxialWaterInteraction::OnControlStatus, this);
    this->node.Subscribe("/coaxial_uav/rotor_water/config",
        &CoaxialWaterInteraction::OnConfig, this);
    this->statusPub = this->node.Advertise<gz::msgs::StringMsg>(
        "/coaxial_uav/rotor_water/status");
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
    if (cfg.movingTargetEnabled && cfg.landingMissionActive &&
        cfg.landingSurfaceMode == "water")
    {
      cfg.waterCurrent.X(cfg.movingTargetVelocity.X());
      cfg.waterCurrent.Y(cfg.movingTargetVelocity.Y());
    }

    const auto upperVelocity = this->upperJoint.Velocity(_ecm);
    const auto lowerVelocity = this->lowerJoint.Velocity(_ecm);
    const double upperOmega = upperVelocity && !upperVelocity->empty() ?
        std::abs(upperVelocity->front()) : 0.0;
    const double lowerOmega = lowerVelocity && !lowerVelocity->empty() ?
        std::abs(lowerVelocity->front()) : 0.0;

    const double upperSquared = upperOmega * upperOmega;
    const double lowerSquared = lowerOmega * lowerOmega;
    const double squaredSum = upperSquared + lowerSquared;
    const bool rotorActive = cfg.rotorInterferenceEnabled &&
        upperOmega >= cfg.minimumRotorOmega &&
        lowerOmega >= cfg.minimumRotorOmega && squaredSum > 1e-9;
    const double overlap = rotorActive ?
        2.0 * std::min(upperSquared, lowerSquared) / squaredSum : 0.0;
    const double targetLoss = cfg.maxThrustLoss * overlap;
    const double inflowAlpha =
        1.0 - std::exp(-dt / std::max(cfg.inflowTimeConstant, 0.01));
    this->filteredInterferenceLoss +=
        inflowAlpha * (targetLoss - this->filteredInterferenceLoss);
    const double baseThrust = cfg.thrustCoeff * squaredSum;
    const double thrustCorrection = rotorActive ?
        -this->filteredInterferenceLoss * baseThrust : 0.0;
    const double baseYawTorque = cfg.momentConstant * cfg.thrustCoeff *
        (lowerSquared - upperSquared);
    const double yawTorqueCorrection = rotorActive ?
        -this->filteredInterferenceLoss * baseYawTorque : 0.0;

    gz::math::Vector3d rotorForceWorld;
    gz::math::Vector3d rotorTorqueWorld;
    if (rotorActive)
    {
      rotorForceWorld = pose->Rot().RotateVector(
          gz::math::Vector3d(0.0, 0.0, thrustCorrection));
      rotorTorqueWorld = pose->Rot().RotateVector(
          gz::math::Vector3d(0.0, 0.0, yawTorqueCorrection));
    }

    gz::math::Vector3d waterForceWorld;
    gz::math::Vector3d waterTorqueWorld;
    double leftFraction = 0.0;
    double rightFraction = 0.0;
    double totalBuoyancy = 0.0;
    double totalSlamming = 0.0;
    if (cfg.hydrodynamicsEnabled)
    {
      const auto left = this->FloatWrench(
          cfg, *pose, *linearWorld, *angularWorld, 1.0);
      const auto right = this->FloatWrench(
          cfg, *pose, *linearWorld, *angularWorld, -1.0);
      waterForceWorld = left.forceWorld + right.forceWorld;
      waterTorqueWorld = left.torqueWorld + right.torqueWorld;
      leftFraction = left.submergedFraction;
      rightFraction = right.submergedFraction;
      totalBuoyancy = left.buoyancyN + right.buoyancyN;
      totalSlamming = left.slammingN + right.slammingN;
      waterForceWorld = Limited(
          waterForceWorld, cfg.maxHydrodynamicForce);
      waterTorqueWorld = Limited(
          waterTorqueWorld, 0.5 * cfg.maxHydrodynamicForce);
    }

    const gz::math::Vector3d totalForceWorld =
        rotorForceWorld + waterForceWorld;
    const gz::math::Vector3d totalTorqueWorld =
        rotorTorqueWorld + waterTorqueWorld;
    if (totalForceWorld.SquaredLength() > 0.0 ||
        totalTorqueWorld.SquaredLength() > 0.0)
    {
      this->link.AddWorldWrench(
          _ecm, totalForceWorld, totalTorqueWorld);
    }

    this->PublishStatus(_info, cfg, upperOmega, lowerOmega, overlap,
        baseThrust, thrustCorrection, yawTorqueCorrection, leftFraction,
        rightFraction, totalBuoyancy, totalSlamming, waterForceWorld,
        waterTorqueWorld);
  }

  private: struct Parameters
  {
    bool rotorInterferenceEnabled{true};
    double maxThrustLoss{0.06};
    double inflowTimeConstant{0.12};
    double minimumRotorOmega{20.0};
    double thrustCoeff{2.1610671e-3};
    double momentConstant{0.06478};
    bool hydrodynamicsEnabled{true};
    double vehicleMass{8.2};
    double waterDensity{997.0};
    double waterLevel{0.0};
    double virtualDraft{0.055};
    double floatRadius{0.09};
    double floatLength{1.10};
    double floatLateralOffset{0.34};
    double floatVerticalOffset{-0.24};
    double floatCount{2.0};
    gz::math::Vector3d linearDrag{4.0, 35.0, 80.0};
    gz::math::Vector3d quadraticDrag{0.25, 1.0, 1.1};
    gz::math::Vector3d waterCurrent{0.0, 0.0, 0.0};
    bool movingTargetEnabled{false};
    bool landingMissionActive{false};
    std::string landingSurfaceMode{"water"};
    gz::math::Vector3d movingTargetVelocity{0.0, 0.0, 0.0};
    double slammingGain{35.0};
    double maxHydrodynamicForce{220.0};
  };

  private: struct FloatResult
  {
    gz::math::Vector3d forceWorld;
    gz::math::Vector3d torqueWorld;
    double submergedFraction{0.0};
    double buoyancyN{0.0};
    double slammingN{0.0};
  };

  private: double FloatBottomOffset(const Parameters &_cfg) const
  {
    return _cfg.floatRadius - _cfg.floatVerticalOffset;
  }

  private: double WaterEquilibriumBodyOffset(const Parameters &_cfg) const
  {
    const double requiredArea = _cfg.vehicleMass /
        std::max(_cfg.waterDensity * _cfg.floatCount * _cfg.floatLength,
                 1e-9);
    const double fullArea = kPi * _cfg.floatRadius * _cfg.floatRadius;
    if (requiredArea >= fullArea)
      return this->FloatBottomOffset(_cfg) + _cfg.virtualDraft -
          2.0 * _cfg.floatRadius;
    double low = 0.0;
    double high = 2.0 * _cfg.floatRadius;
    for (int i = 0; i < 48; ++i)
    {
      const double depth = 0.5 * (low + high);
      if (CircularSegmentArea(_cfg.floatRadius, depth) < requiredArea)
        low = depth;
      else
        high = depth;
    }
    return this->FloatBottomOffset(_cfg) + _cfg.virtualDraft -
        0.5 * (low + high);
  }

  private: FloatResult FloatWrench(
      const Parameters &_cfg, const gz::math::Pose3d &_pose,
      const gz::math::Vector3d &_linearWorld,
      const gz::math::Vector3d &_angularWorld, double _side) const
  {
    FloatResult result;
    const gz::math::Vector3d bodyYWorld =
        _pose.Rot().RotateVector(gz::math::Vector3d::UnitY);
    const gz::math::Vector3d bodyZWorld =
        _pose.Rot().RotateVector(gz::math::Vector3d::UnitZ);
    const double projectedRadius = _cfg.floatRadius * std::sqrt(
        bodyYWorld.Z() * bodyYWorld.Z() +
        bodyZWorld.Z() * bodyZWorld.Z());
    const double effectiveSurface = _cfg.waterLevel + _cfg.virtualDraft;
    const double fullArea = kPi * _cfg.floatRadius * _cfg.floatRadius;
    const double sectionLength = 0.5 * _cfg.floatLength;
    double fractionSum = 0.0;
    for (const double xOffset :
        {-0.25 * _cfg.floatLength, 0.25 * _cfg.floatLength})
    {
      const gz::math::Vector3d centerBody(
          xOffset, _side * _cfg.floatLateralOffset,
          _cfg.floatVerticalOffset);
      const gz::math::Vector3d armWorld =
          _pose.Rot().RotateVector(centerBody);
      const gz::math::Vector3d centerWorld = _pose.Pos() + armWorld;
      const double bottomZ = centerWorld.Z() - projectedRadius;
      const double depth = Clamp(
          effectiveSurface - bottomZ, 0.0, 2.0 * _cfg.floatRadius);
      const double segmentArea =
          CircularSegmentArea(_cfg.floatRadius, depth);
      const double submergedFraction = fullArea > 0.0 ?
          segmentArea / fullArea : 0.0;
      fractionSum += submergedFraction;
      if (submergedFraction <= 0.0)
        continue;

      const double volume = segmentArea * sectionLength;
      const double buoyancyN = _cfg.waterDensity * 9.8 * volume;
      const gz::math::Vector3d pointVelocityWorld =
          _linearWorld + _angularWorld.Cross(armWorld);
      const gz::math::Vector3d relativeWorld =
          pointVelocityWorld - _cfg.waterCurrent;
      const gz::math::Vector3d relativeBody =
          _pose.Rot().RotateVectorReverse(relativeWorld);
      const double wetScale = std::sqrt(submergedFraction);
      const gz::math::Vector3d referenceArea(
          0.5 * kPi * _cfg.floatRadius * _cfg.floatRadius,
          2.0 * _cfg.floatRadius * sectionLength,
          2.0 * _cfg.floatRadius * sectionLength);
      gz::math::Vector3d dragBody;
      for (int axis = 0; axis < 3; ++axis)
      {
        dragBody[axis] =
            -0.5 * wetScale * _cfg.linearDrag[axis] *
                relativeBody[axis] -
            0.5 * _cfg.waterDensity * _cfg.quadraticDrag[axis] *
                referenceArea[axis] * submergedFraction *
                std::abs(relativeBody[axis]) * relativeBody[axis];
      }
      const double entrySpeed = std::max(0.0, -pointVelocityWorld.Z());
      const double entryEnvelope =
          Clamp(depth / std::max(0.25 * _cfg.floatRadius, 1e-6),
                0.0, 1.0) *
          Clamp((2.0 * _cfg.floatRadius - depth) /
                std::max(0.5 * _cfg.floatRadius, 1e-6), 0.0, 1.0);
      const double slammingN =
          0.5 * _cfg.slammingGain * entryEnvelope * entrySpeed;
      const gz::math::Vector3d dragWorld =
          _pose.Rot().RotateVector(dragBody);
      const gz::math::Vector3d verticalForce(
          0.0, 0.0, buoyancyN + slammingN);
      const gz::math::Vector3d metacentricArmWorld =
          _pose.Rot().RotateVector(gz::math::Vector3d(
              xOffset, _side * _cfg.floatLateralOffset, 0.0));
      result.forceWorld += dragWorld + verticalForce;
      result.torqueWorld +=
          armWorld.Cross(dragWorld) +
          metacentricArmWorld.Cross(verticalForce);
      result.buoyancyN += buoyancyN;
      result.slammingN += slammingN;
    }
    result.submergedFraction = 0.5 * fractionSum;
    return result;
  }

  private: void OnConfig(const gz::msgs::StringMsg &_msg)
  {
    const auto text = _msg.data();
    std::lock_guard<std::mutex> lock(this->mutex);
    auto &cfg = this->parameters;
    cfg.rotorInterferenceEnabled = BoolField(
        text, "rotor_interference_enabled", cfg.rotorInterferenceEnabled);
    cfg.maxThrustLoss = Clamp(NumberField(
        text, "coaxial_max_thrust_loss", cfg.maxThrustLoss), 0.0, 0.35);
    cfg.inflowTimeConstant = Clamp(NumberField(
        text, "coaxial_inflow_time_constant_s",
        cfg.inflowTimeConstant), 0.01, 2.0);
    cfg.hydrodynamicsEnabled = BoolField(
        text, "hydrodynamics_enabled", cfg.hydrodynamicsEnabled);
    cfg.waterDensity = Clamp(NumberField(
        text, "water_density_kg_m3", cfg.waterDensity), 500.0, 1300.0);
    cfg.waterLevel = Clamp(NumberField(
        text, "water_level_z_m", cfg.waterLevel), -2.0, 2.0);
    cfg.virtualDraft = Clamp(NumberField(
        text, "float_virtual_draft_m", cfg.virtualDraft),
        0.0, 2.0 * cfg.floatRadius);
    cfg.linearDrag.X(Clamp(NumberField(
        text, "water_linear_drag_x_n_s_m", cfg.linearDrag.X()), 0.0, 1000.0));
    cfg.linearDrag.Y(Clamp(NumberField(
        text, "water_linear_drag_y_n_s_m", cfg.linearDrag.Y()), 0.0, 1000.0));
    cfg.linearDrag.Z(Clamp(NumberField(
        text, "water_linear_drag_z_n_s_m", cfg.linearDrag.Z()), 0.0, 1000.0));
    cfg.quadraticDrag.X(Clamp(NumberField(
        text, "water_quadratic_drag_x", cfg.quadraticDrag.X()), 0.0, 5.0));
    cfg.quadraticDrag.Y(Clamp(NumberField(
        text, "water_quadratic_drag_y", cfg.quadraticDrag.Y()), 0.0, 5.0));
    cfg.quadraticDrag.Z(Clamp(NumberField(
        text, "water_quadratic_drag_z", cfg.quadraticDrag.Z()), 0.0, 5.0));
    cfg.waterCurrent.X(Clamp(NumberField(
        text, "water_current_x_m_s", cfg.waterCurrent.X()), -5.0, 5.0));
    cfg.waterCurrent.Y(Clamp(NumberField(
        text, "water_current_y_m_s", cfg.waterCurrent.Y()), -5.0, 5.0));
    cfg.waterCurrent.Z(Clamp(NumberField(
        text, "water_current_z_m_s", cfg.waterCurrent.Z()), -2.0, 2.0));
    cfg.movingTargetEnabled = BoolField(
        text, "landing_moving_target_enabled", cfg.movingTargetEnabled);
    const bool nextMissionActive = BoolField(
        text, "landing_mission_active", cfg.landingMissionActive);
    if (BoolField(text, "landing_start", false) || nextMissionActive)
    {
      cfg.landingMissionActive = true;
      this->landingControllerActiveSeen = false;
    }
    else if (!nextMissionActive)
    {
      cfg.landingMissionActive = false;
      this->landingControllerActiveSeen = false;
    }
    const auto surfaceMode = StringField(
        text, "landing_surface_mode", cfg.landingSurfaceMode);
    cfg.landingSurfaceMode = surfaceMode == "platform" ?
        "platform" : "water";
    cfg.movingTargetVelocity.X(Clamp(NumberField(
        text, "landing_target_vx_m_s", cfg.movingTargetVelocity.X()),
        -2.0, 2.0));
    cfg.movingTargetVelocity.Y(Clamp(NumberField(
        text, "landing_target_vy_m_s", cfg.movingTargetVelocity.Y()),
        -2.0, 2.0));
    cfg.slammingGain = Clamp(NumberField(
        text, "water_slamming_gain_n_s_m", cfg.slammingGain), 0.0, 500.0);
  }

  private: void OnControlStatus(const gz::msgs::StringMsg &_msg)
  {
    const auto text = _msg.data();
    std::lock_guard<std::mutex> lock(this->mutex);
    const bool controllerLandingActive = BoolField(
        text, "landing_active", this->parameters.landingMissionActive);
    if (controllerLandingActive)
    {
      this->parameters.landingMissionActive = true;
      this->landingControllerActiveSeen = true;
    }
    else if (this->landingControllerActiveSeen)
    {
      this->parameters.landingMissionActive = false;
      this->landingControllerActiveSeen = false;
    }
  }

  private: void PublishStatus(const gz::sim::UpdateInfo &_info,
      const Parameters &_cfg, double _upperOmega, double _lowerOmega,
      double _overlap, double _baseThrust, double _thrustCorrection,
      double _yawTorqueCorrection, double _leftFraction,
      double _rightFraction, double _buoyancy, double _slamming,
      const gz::math::Vector3d &_waterForce,
      const gz::math::Vector3d &_waterTorque)
  {
    const double simTime = std::chrono::duration<double>(_info.simTime).count();
    if (simTime - this->lastStatusTime < 0.003)
      return;
    this->lastStatusTime = simTime;

    std::ostringstream out;
    const double floatBottomOffset = this->FloatBottomOffset(_cfg);
    const double equilibriumBodyOffset =
        this->WaterEquilibriumBodyOffset(_cfg);
    out << "{\"rotor_interference_enabled\":"
        << (_cfg.rotorInterferenceEnabled ? "true" : "false")
        << ",\"rotor_interference_active\":"
        << ((_overlap > 0.0) ? "true" : "false")
        << ",\"upper_rotor_omega_rad_s\":" << _upperOmega
        << ",\"lower_rotor_omega_rad_s\":" << _lowerOmega
        << ",\"coaxial_overlap\":" << _overlap
        << ",\"coaxial_loss_fraction\":" << this->filteredInterferenceLoss
        << ",\"base_rotor_thrust_n\":" << _baseThrust
        << ",\"rotor_thrust_correction_n\":" << _thrustCorrection
        << ",\"rotor_yaw_torque_correction_nm\":" << _yawTorqueCorrection
        << ",\"hydrodynamics_enabled\":"
        << (_cfg.hydrodynamicsEnabled ? "true" : "false")
        << ",\"moving_target_current_coupled\":"
        << ((_cfg.movingTargetEnabled && _cfg.landingMissionActive &&
             _cfg.landingSurfaceMode == "water") ? "true" : "false")
        << ",\"moving_target_current_coupling_available\":true"
        << ",\"landing_surface_mode\":\""
        << _cfg.landingSurfaceMode << "\""
        << ",\"vehicle_geometry_version\":\"float_geometry_v1\""
        << ",\"float_bottom_offset_m\":" << floatBottomOffset
        << ",\"float_footprint_half_length_m\":"
        << 0.5 * _cfg.floatLength
        << ",\"float_footprint_half_width_m\":"
        << std::abs(_cfg.floatLateralOffset) + _cfg.floatRadius
        << ",\"water_equilibrium_body_offset_m\":"
        << equilibriumBodyOffset
        << ",\"float_radius_m\":" << _cfg.floatRadius
        << ",\"float_length_m\":" << _cfg.floatLength
        << ",\"float_lateral_offset_m\":" << _cfg.floatLateralOffset
        << ",\"float_vertical_offset_m\":" << _cfg.floatVerticalOffset
        << ",\"effective_water_current_world_m_s\":["
        << _cfg.waterCurrent.X() << "," << _cfg.waterCurrent.Y() << ","
        << _cfg.waterCurrent.Z() << "]"
        << ",\"water_contact\":"
        << ((_leftFraction > 0.0 || _rightFraction > 0.0) ? "true" : "false")
        << ",\"left_float_submerged_fraction\":" << _leftFraction
        << ",\"right_float_submerged_fraction\":" << _rightFraction
        << ",\"buoyancy_n\":" << _buoyancy
        << ",\"slamming_force_n\":" << _slamming
        << ",\"water_force_world_n\":["
        << _waterForce.X() << "," << _waterForce.Y() << ","
        << _waterForce.Z() << "]"
        << ",\"water_torque_world_nm\":["
        << _waterTorque.X() << "," << _waterTorque.Y() << ","
        << _waterTorque.Z() << "]"
        << ",\"water_level_z_m\":" << _cfg.waterLevel
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
  private: double filteredInterferenceLoss{0.0};
  private: double lastStatusTime{-1.0};
  private: bool landingControllerActiveSeen{false};
};
}

GZ_ADD_PLUGIN(coaxial_uav::CoaxialWaterInteraction,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(coaxial_uav::CoaxialWaterInteraction,
    "coaxial_uav::CoaxialWaterInteraction")
