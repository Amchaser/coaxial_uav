#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <random>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
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

double WrapPi(double _angle)
{
  while (_angle > M_PI)
    _angle -= 2.0 * M_PI;
  while (_angle < -M_PI)
    _angle += 2.0 * M_PI;
  return _angle;
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

struct Pid
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double limit{0.0};
  double integralLimit{0.0};
  double integral{0.0};

  double Update(double _error, double _rate, double _dt)
  {
    this->integral = Clamp(this->integral + _error * _dt,
                           -this->integralLimit, this->integralLimit);
    const double output = this->kp * _error + this->ki * this->integral -
        this->kd * _rate;
    return Clamp(output, -this->limit, this->limit);
  }

  double UpdateAntiWindup(double _error, double _rate, double _dt)
  {
    const double candidateIntegral = Clamp(this->integral + _error * _dt,
        -this->integralLimit, this->integralLimit);
    const double candidateOutput = this->kp * _error +
        this->ki * candidateIntegral - this->kd * _rate;
    if (std::abs(candidateOutput) <= this->limit ||
        (candidateOutput > this->limit && _error < 0.0) ||
        (candidateOutput < -this->limit && _error > 0.0))
    {
      this->integral = candidateIntegral;
    }
    const double output = this->kp * _error + this->ki * this->integral -
        this->kd * _rate;
    return Clamp(output, -this->limit, this->limit);
  }
};
}

class CoaxialPidController:
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

    this->mass = SdfDouble(_sdf, "mass_kg", this->mass);
    this->gravity = SdfDouble(_sdf, "gravity_m_s2", this->gravity);
    this->thrustCoeff = SdfDouble(_sdf, "thrust_coeff", this->thrustCoeff);
    this->rotorInterferenceEnabled = SdfBool(_sdf,
        "rotor_interference_enabled", this->rotorInterferenceEnabled);
    this->coaxialMaxThrustLoss = Clamp(SdfDouble(_sdf,
        "coaxial_max_thrust_loss", this->coaxialMaxThrustLoss), 0.0, 0.35);
    this->targetZ = SdfDouble(_sdf, "target_z_m", this->targetZ);
    this->maxOmega = SdfDouble(_sdf, "max_omega_rad_s", this->maxOmega);
    this->attitudeSetpointRateLimit = Clamp(SdfDouble(_sdf,
        "attitude_setpoint_rate_limit_rad_s",
        this->attitudeSetpointRateLimit), 0.0, 20.0);
    this->velocityControlEnabled = SdfBool(_sdf,
        "velocity_control_enabled", this->velocityControlEnabled);
    this->targetVx = SdfDouble(_sdf, "target_vx_m_s", this->targetVx);
    this->targetVy = SdfDouble(_sdf, "target_vy_m_s", this->targetVy);
    this->velocityTiltLimit = Clamp(SdfDouble(_sdf,
        "velocity_tilt_limit_rad", this->velocityTiltLimit), 0.0, 0.785398);
    this->velocityAccelLimit = std::abs(SdfDouble(_sdf,
        "velocity_accel_limit_m_s2", this->velocityAccelLimit));
    this->velocityX.kp = SdfDouble(_sdf, "velocity_x_kp", this->velocityX.kp);
    this->velocityX.ki = SdfDouble(_sdf, "velocity_x_ki", this->velocityX.ki);
    this->velocityX.limit = SdfDouble(_sdf,
        "velocity_x_limit", this->velocityX.limit);
    this->velocityX.integralLimit = SdfDouble(_sdf,
        "velocity_x_integral_limit", this->velocityX.integralLimit);
    this->velocityY.kp = SdfDouble(_sdf, "velocity_y_kp", this->velocityY.kp);
    this->velocityY.ki = SdfDouble(_sdf, "velocity_y_ki", this->velocityY.ki);
    this->velocityY.limit = SdfDouble(_sdf,
        "velocity_y_limit", this->velocityY.limit);
    this->velocityY.integralLimit = SdfDouble(_sdf,
        "velocity_y_integral_limit", this->velocityY.integralLimit);
    this->positionControlEnabled = SdfBool(_sdf,
        "position_control_enabled", this->positionControlEnabled);
    this->targetX = SdfDouble(_sdf, "target_x_m", this->targetX);
    this->targetY = SdfDouble(_sdf, "target_y_m", this->targetY);
    this->positionVelocityLimit = std::abs(SdfDouble(_sdf,
        "position_velocity_limit_m_s", this->positionVelocityLimit));
    this->positionX.kp = SdfDouble(_sdf, "position_x_kp", this->positionX.kp);
    this->positionX.ki = SdfDouble(_sdf, "position_x_ki", this->positionX.ki);
    this->positionX.kd = SdfDouble(_sdf, "position_x_kd", this->positionX.kd);
    this->positionX.limit = SdfDouble(_sdf,
        "position_x_limit", this->positionX.limit);
    this->positionX.integralLimit = SdfDouble(_sdf,
        "position_x_integral_limit", this->positionX.integralLimit);
    this->positionY.kp = SdfDouble(_sdf, "position_y_kp", this->positionY.kp);
    this->positionY.ki = SdfDouble(_sdf, "position_y_ki", this->positionY.ki);
    this->positionY.kd = SdfDouble(_sdf, "position_y_kd", this->positionY.kd);
    this->positionY.limit = SdfDouble(_sdf,
        "position_y_limit", this->positionY.limit);
    this->positionY.integralLimit = SdfDouble(_sdf,
        "position_y_integral_limit", this->positionY.integralLimit);
    this->enabled = SdfBool(_sdf, "enabled", this->enabled);
    this->height.kp = SdfDouble(_sdf, "height_kp", this->height.kp);
    this->height.ki = SdfDouble(_sdf, "height_ki", this->height.ki);
    this->height.kd = SdfDouble(_sdf, "height_kd", this->height.kd);
    this->height.limit = SdfDouble(_sdf, "height_limit", this->height.limit);
    this->height.integralLimit = SdfDouble(_sdf, "height_integral_limit",
        this->height.integralLimit);
    const double attKp = SdfDouble(_sdf, "att_kp", this->roll.kp);
    const double attKd = SdfDouble(_sdf, "att_kd", this->roll.kd);
    const double attLimit = SdfDouble(_sdf, "att_limit", this->roll.limit);
    this->roll.kp = SdfDouble(_sdf, "roll_kp", attKp);
    this->pitch.kp = SdfDouble(_sdf, "pitch_kp", attKp);
    this->roll.kd = SdfDouble(_sdf, "roll_kd", attKd);
    this->pitch.kd = SdfDouble(_sdf, "pitch_kd", attKd);
    this->roll.limit = SdfDouble(_sdf, "roll_limit", attLimit);
    this->pitch.limit = SdfDouble(_sdf, "pitch_limit", attLimit);
    this->yaw.kp = SdfDouble(_sdf, "yaw_kp", this->yaw.kp);
    this->yaw.kd = SdfDouble(_sdf, "yaw_kd", this->yaw.kd);
    this->yaw.limit = SdfDouble(_sdf, "yaw_limit", this->yaw.limit);
    this->yawLargeKp =
        SdfDouble(_sdf, "yaw_large_signal_kp", this->yawLargeKp);
    this->yawLargeKd =
        SdfDouble(_sdf, "yaw_large_signal_kd", this->yawLargeKd);
    this->yawScheduleStart =
        SdfDouble(_sdf, "yaw_schedule_start_rad", this->yawScheduleStart);
    this->yawScheduleEnd =
        SdfDouble(_sdf, "yaw_schedule_end_rad", this->yawScheduleEnd);
    this->nonidealitiesEnabled = SdfBool(_sdf,
        "nonidealities_enabled", this->nonidealitiesEnabled);
    this->attitudeNoiseStd = std::abs(SdfDouble(_sdf,
        "attitude_noise_std_rad", this->attitudeNoiseStd));
    this->gyroNoiseStd = std::abs(SdfDouble(_sdf,
        "gyro_noise_std_rad_s", this->gyroNoiseStd));
    this->attitudeBiasStd = std::abs(SdfDouble(_sdf,
        "attitude_bias_std_rad", this->attitudeBiasStd));
    this->gyroBiasStd = std::abs(SdfDouble(_sdf,
        "gyro_bias_std_rad_s", this->gyroBiasStd));
    this->positionNoiseStd = std::abs(SdfDouble(_sdf,
        "position_noise_std_m", this->positionNoiseStd));
    this->velocityNoiseStd = std::abs(SdfDouble(_sdf,
        "velocity_noise_std_m_s", this->velocityNoiseStd));
    this->controlDelay = Clamp(SdfDouble(_sdf,
        "control_delay_s", this->controlDelay), 0.0, 0.5);
    this->motorTimeConstant = Clamp(SdfDouble(_sdf,
        "motor_time_constant_s", this->motorTimeConstant), 0.0, 1.0);
    this->motorRateLimit = std::abs(SdfDouble(_sdf,
        "motor_rate_limit_rad_s2", this->motorRateLimit));
    this->motorEffectiveness = Clamp(SdfDouble(_sdf,
        "motor_effectiveness", this->motorEffectiveness), 0.5, 1.0);
    this->nonidealitiesSeed = static_cast<std::uint32_t>(std::max(0.0,
        SdfDouble(_sdf, "nonidealities_seed", this->nonidealitiesSeed)));
    this->ResetNoiseGenerator();
    this->filteredTargetRoll = this->targetRoll;
    this->filteredTargetPitch = this->targetPitch;
    this->filteredTargetYaw = this->targetYaw;

    this->link.EnableVelocityChecks(_ecm, true);
    this->upperJoint.EnableVelocityCheck(_ecm, true);
    this->lowerJoint.EnableVelocityCheck(_ecm, true);
    this->node.Subscribe("/coaxial_uav/control/config",
        &CoaxialPidController::OnConfig, this);
    this->node.Subscribe("/coaxial_uav/rotor_water/status",
        &CoaxialPidController::OnWaterStatus, this);
    this->node.Subscribe("/coaxial_uav/landing/target/status",
        &CoaxialPidController::OnLandingTargetStatus, this);
    this->statusPub =
        this->node.Advertise<gz::msgs::StringMsg>("/coaxial_uav/control/status");
    this->landingStatusPub =
        this->node.Advertise<gz::msgs::StringMsg>("/coaxial_uav/landing/status");
    this->motorCommandPub = this->node.Advertise<gz::msgs::Actuators>(
        "/coaxial_uav/gazebo/command/motor_speed");
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
    const auto linearVel = this->link.WorldLinearVelocity(_ecm);
    const auto angularVel = this->link.WorldAngularVelocity(_ecm);
    if (!pose || !linearVel || !angularVel)
      return;
    const auto euler = pose->Rot().Euler();
    const auto bodyRates = pose->Rot().Inverse().RotateVector(*angularVel);

    const double simTime =
        std::chrono::duration<double>(_info.simTime).count();
    ConfigSnapshot cfg;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      cfg.nonidealitiesEnabled = this->nonidealitiesEnabled;
      cfg.controlDelay = this->controlDelay;
      cfg.motorTimeConstant = this->motorTimeConstant;
      cfg.motorRateLimit = this->motorRateLimit;
      cfg.motorEffectiveness = this->motorEffectiveness;
      cfg.measuredPosition = pose->Pos();
      cfg.measuredLinearVelocity = *linearVel;
      cfg.measuredEuler = euler;
      cfg.measuredBodyRates = bodyRates;
      if (cfg.nonidealitiesEnabled)
      {
        cfg.measuredPosition += gz::math::Vector3d(
            this->Noise(this->positionNoiseStd),
            this->Noise(this->positionNoiseStd),
            this->Noise(this->positionNoiseStd));
        cfg.measuredLinearVelocity += gz::math::Vector3d(
            this->Noise(this->velocityNoiseStd),
            this->Noise(this->velocityNoiseStd),
            this->Noise(this->velocityNoiseStd));
        cfg.measuredEuler += this->attitudeBias + gz::math::Vector3d(
            this->Noise(this->attitudeNoiseStd),
            this->Noise(this->attitudeNoiseStd),
            this->Noise(this->attitudeNoiseStd));
        cfg.measuredBodyRates += this->gyroBias + gz::math::Vector3d(
            this->Noise(this->gyroNoiseStd),
            this->Noise(this->gyroNoiseStd),
            this->Noise(this->gyroNoiseStd));
      }
      this->filteredBuoyancy +=
          (1.0 - std::exp(-dt / 0.08)) *
          (Clamp(this->waterBuoyancy, 0.0, this->mass * this->gravity) -
           this->filteredBuoyancy);
      this->UpdateLandingLocked(simTime, dt, cfg.measuredPosition,
          cfg.measuredLinearVelocity, cfg.measuredEuler,
          cfg.measuredBodyRates);
      double commandRoll = this->targetRoll;
      double commandPitch = this->targetPitch;
      double commandVx = this->targetVx;
      double commandVy = this->targetVy;
      double accelWorldX = 0.0;
      double accelWorldY = 0.0;
      if (this->positionControlEnabled && this->enabled)
      {
        const double errorX = this->targetX - cfg.measuredPosition.X();
        const double errorY = this->targetY - cfg.measuredPosition.Y();
        const bool targetTrackingPhase =
            this->landingState == "APPROACH" ||
            this->landingState == "ALIGN" ||
            this->landingState == "HIGH_HOVER" ||
            this->landingState == "SLOW_DESCENT" ||
            this->landingState == "NEAR_WATER";
        const bool targetFeedforwardEnabled =
            this->landingActive && this->landingMovingTargetEnabled &&
            targetTrackingPhase && !this->landingTargetTrackingFrozen;
        const double feedforwardVx =
            targetFeedforwardEnabled ? this->landingTargetVx : 0.0;
        const double feedforwardVy =
            targetFeedforwardEnabled ? this->landingTargetVy : 0.0;
        const double previousIntegralX = this->positionX.integral;
        const double previousIntegralY = this->positionY.integral;
        commandVx = feedforwardVx + this->positionX.UpdateAntiWindup(
            errorX, cfg.measuredLinearVelocity.X() - feedforwardVx, dt);
        commandVy = feedforwardVy + this->positionY.UpdateAntiWindup(
            errorY, cfg.measuredLinearVelocity.Y() - feedforwardVy, dt);
        const double speedMagnitude = std::hypot(commandVx, commandVy);
        if (this->positionVelocityLimit > 0.0 &&
            speedMagnitude > this->positionVelocityLimit)
        {
          const double scale = this->positionVelocityLimit / speedMagnitude;
          commandVx *= scale;
          commandVy *= scale;
          if (commandVx * errorX > 0.0)
            this->positionX.integral = previousIntegralX;
          if (commandVy * errorY > 0.0)
            this->positionY.integral = previousIntegralY;
        }
      }
      if ((this->velocityControlEnabled || this->positionControlEnabled) &&
          this->enabled)
      {
        const double errorVx = commandVx - cfg.measuredLinearVelocity.X();
        const double errorVy = commandVy - cfg.measuredLinearVelocity.Y();
        const double previousIntegralX = this->velocityX.integral;
        const double previousIntegralY = this->velocityY.integral;
        accelWorldX = this->velocityX.UpdateAntiWindup(errorVx, 0.0, dt);
        accelWorldY = this->velocityY.UpdateAntiWindup(errorVy, 0.0, dt);
        const double vectorLimit = std::min(this->velocityAccelLimit,
            this->gravity * std::tan(this->velocityTiltLimit));
        const double accelMagnitude = std::hypot(accelWorldX, accelWorldY);
        if (vectorLimit > 0.0 && accelMagnitude > vectorLimit)
        {
          const double scale = vectorLimit / accelMagnitude;
          accelWorldX *= scale;
          accelWorldY *= scale;
          if (accelWorldX * errorVx > 0.0)
            this->velocityX.integral = previousIntegralX;
          if (accelWorldY * errorVy > 0.0)
            this->velocityY.integral = previousIntegralY;
        }
        const double yaw = cfg.measuredEuler.Z();
        const double accelBodyX =
            std::cos(yaw) * accelWorldX + std::sin(yaw) * accelWorldY;
        const double accelBodyY =
            -std::sin(yaw) * accelWorldX + std::cos(yaw) * accelWorldY;
        commandPitch = Clamp(std::atan2(accelBodyX, this->gravity),
            -this->velocityTiltLimit, this->velocityTiltLimit);
        commandRoll = Clamp(std::atan2(-accelBodyY, this->gravity),
            -this->velocityTiltLimit, this->velocityTiltLimit);
      }
      const double maxSetpointStep = this->attitudeSetpointRateLimit * dt;
      const auto updateSetpoint = [maxSetpointStep](
          double _filtered, double _command)
      {
        const double delta = WrapPi(_command - _filtered);
        if (maxSetpointStep <= 0.0)
          return _filtered + delta;
        return _filtered + Clamp(delta, -maxSetpointStep, maxSetpointStep);
      };
      this->filteredTargetRoll = updateSetpoint(
          this->filteredTargetRoll, commandRoll);
      this->filteredTargetPitch = updateSetpoint(
          this->filteredTargetPitch, commandPitch);
      this->filteredTargetYaw = updateSetpoint(
          this->filteredTargetYaw, this->targetYaw);
      cfg.enabled = this->enabled;
      cfg.targetZ = this->targetZ;
      cfg.commandRoll = commandRoll;
      cfg.commandPitch = commandPitch;
      cfg.commandYaw = this->targetYaw;
      cfg.targetRoll = this->filteredTargetRoll;
      cfg.targetPitch = this->filteredTargetPitch;
      cfg.targetYaw = this->filteredTargetYaw;
      cfg.maxOmega = this->maxOmega;
      cfg.rotorInterferenceEnabled = this->rotorInterferenceEnabled;
      cfg.coaxialMaxThrustLoss = this->coaxialMaxThrustLoss;
      cfg.height = this->height;
      cfg.roll = this->roll;
      cfg.pitch = this->pitch;
      cfg.yaw = this->yaw;
      cfg.yawLargeKp = this->yawLargeKp;
      cfg.yawLargeKd = this->yawLargeKd;
      cfg.yawScheduleStart = this->yawScheduleStart;
      cfg.yawScheduleEnd = this->yawScheduleEnd;
      cfg.velocityControlEnabled = this->velocityControlEnabled;
      cfg.targetVx = commandVx;
      cfg.targetVy = commandVy;
      cfg.velocityAccelX = accelWorldX;
      cfg.velocityAccelY = accelWorldY;
      cfg.velocityAccelLimit = this->velocityAccelLimit;
      cfg.positionControlEnabled = this->positionControlEnabled;
      cfg.targetX = this->targetX;
      cfg.targetY = this->targetY;
      cfg.positionVelocityLimit = this->positionVelocityLimit;
      cfg.targetVz = this->landingActive ? this->landingTargetVz : 0.0;
      cfg.filteredBuoyancy = this->filteredBuoyancy;
      cfg.waterContact = this->waterContact;
      cfg.leftSubmerged = this->leftSubmerged;
      cfg.rightSubmerged = this->rightSubmerged;
      cfg.slammingForce = this->waterSlammingForce;
      cfg.landingState = this->landingState;
      cfg.landingActive = this->landingActive;
      cfg.landingStateStart = this->landingStateStart;
      cfg.floatSignedClearance = cfg.measuredPosition.Z() -
          this->landingFloatBottomOffset -
          this->landingSurfaceZ;
      cfg.floatClearance = std::max(0.0, cfg.floatSignedClearance);
      cfg.landingPeakImpact = this->landingPeakImpact;
      cfg.landingImpactImpulse = this->landingImpactImpulse;
      cfg.touchdownVz = this->landingTouchdownVz;
      cfg.landingTargetX = this->landingTargetX;
      cfg.landingTargetY = this->landingTargetY;
      cfg.landingTargetYaw = this->landingTargetYaw;
      cfg.landingMissionHoverZ = this->landingMissionHoverZ;
      cfg.landingStartedOnWater = this->landingStartedOnWater;
      cfg.landingSurfaceMode = this->landingSurfaceMode;
      cfg.landingSurfaceZ = this->landingSurfaceZ;
      cfg.landingPlatformTopOffset = this->landingPlatformTopOffset;
      cfg.landingVehicleGeometryReady = this->landingVehicleGeometryReady;
      cfg.landingSurfaceGeometryReady = this->landingSurfaceGeometryReady;
      cfg.landingFloatBottomOffset = this->landingFloatBottomOffset;
      cfg.landingWaterEquilibriumBodyOffset =
          this->landingWaterEquilibriumBodyOffset;
      cfg.landingPlatformSafeHalfLength = this->PlatformSafeHalfLength();
      cfg.landingPlatformSafeHalfWidth = this->PlatformSafeHalfWidth();
      cfg.landingContactMinClearance = this->landingContactMinClearance;
      cfg.landingContactMaxClearance = this->landingContactMaxClearance;
      cfg.landingPlatformAvailable = this->landingPlatformAvailable;
      cfg.landingPlatformContact = this->PlatformContact(
          cfg.measuredPosition);
      cfg.landingApproachSpeed = this->landingApproachSpeed;
      cfg.landingCruiseSpeed = this->landingCruiseSpeed;
      cfg.landingDepartureHorizontalSpeedLimit =
          this->landingDepartureHorizontalSpeedLimit;
      cfg.landingNearHorizontalSpeedLimit =
          this->landingNearHorizontalSpeedLimit;
      cfg.landingMovingTargetCorrectionReserve =
          this->landingMovingTargetCorrectionReserve;
      cfg.landingApproachBrakingAccel = this->landingApproachBrakingAccel;
      cfg.landingDepartureTiltLimit = this->landingDepartureTiltLimit;
      cfg.landingApproachTiltLimit = this->landingApproachTiltLimit;
      cfg.landingNearTiltLimit = this->landingNearTiltLimit;
      cfg.landingWarningTilt = this->landingWarningTilt;
      cfg.landingAbortTilt = this->landingAbortTilt;
      cfg.landingApproachAbortTilt = this->landingApproachAbortTilt;
      cfg.landingAbortPositionError = this->landingAbortPositionError;
      cfg.landingNearMaxDescentSpeed = this->landingNearMaxDescentSpeed;
      cfg.landingGoAroundHeight = this->landingGoAroundHeight;
      cfg.landingContactLossGrace = this->landingContactLossGrace;
      cfg.landingContactSubmergedFraction =
          this->landingContactSubmergedFraction;
      cfg.landingMovingTargetEnabled = this->landingMovingTargetEnabled;
      cfg.landingTargetVx = this->landingTargetVx;
      cfg.landingTargetVy = this->landingTargetVy;
      cfg.landingTargetYawRate = this->landingTargetYawRate;
      cfg.landingTargetHealthy = this->landingTargetHealthy;
      cfg.landingTargetStatusAge = this->landingTargetStatusAge;
      cfg.touchdownHorizontalError =
          this->landingTouchdownHorizontalError;
      cfg.touchdownRelativeSpeed = this->landingTouchdownRelativeSpeed;
      cfg.touchdownYawError = this->landingTouchdownYawError;
      cfg.dualContactDelay = this->landingDualContactDelay;
      cfg.abortReason = this->landingAbortReason;
      cfg.abortTriggerState = this->landingAbortTriggerState;
      cfg.abortMeasuredValue = this->landingAbortMeasuredValue;
      cfg.abortLimitValue = this->landingAbortLimitValue;
      cfg.spoolDown = this->landingState == "SPOOL_DOWN";
      cfg.spoolOmega = this->landingSpoolOmega;
    }

    const auto upperVelocity = this->upperJoint.Velocity(_ecm);
    const auto lowerVelocity = this->lowerJoint.Velocity(_ecm);
    const double upperOmega = upperVelocity && !upperVelocity->empty() ?
        upperVelocity->front() : 0.0;
    const double lowerOmega = lowerVelocity && !lowerVelocity->empty() ?
        lowerVelocity->front() : 0.0;

    if (!cfg.enabled)
    {
      this->yawLargeSignalMode = false;
      this->commandQueue.clear();
      this->lastDelayedCommand = ActuatorCommand();
      this->appliedMotorOmega = 0.0;
      this->PublishMotorSpeeds(0.0);
      this->PublishStatus(_info, *pose, *linearVel, bodyRates, cfg,
          0.0, 0.0, upperOmega, lowerOmega,
          gz::math::Vector3d::Zero, gz::math::Vector3d::Zero, false);
      return;
    }

    const double zError = cfg.targetZ - cfg.measuredPosition.Z();
    const double thrustDelta = cfg.height.Update(
        zError, cfg.measuredLinearVelocity.Z() - cfg.targetVz, dt);
    const double thrustAxisVertical = std::max(0.5,
        pose->Rot().RotateVector(gz::math::Vector3d::UnitZ).Z());
    const double effectiveThrustCoeff = this->thrustCoeff *
        (cfg.rotorInterferenceEnabled ?
            1.0 - cfg.coaxialMaxThrustLoss : 1.0);
    const double thrust = Clamp(
        (this->mass * this->gravity - cfg.filteredBuoyancy + thrustDelta) /
            thrustAxisVertical,
        0.0, 2.0 * effectiveThrustCoeff * cfg.maxOmega * cfg.maxOmega);
    double omega = std::sqrt(
        std::max(0.0, thrust / (2.0 * effectiveThrustCoeff)));
    if (cfg.spoolDown)
      omega = cfg.spoolOmega;

    const double rollTorque = cfg.roll.Update(
        WrapPi(cfg.targetRoll - cfg.measuredEuler.X()),
        cfg.measuredBodyRates.X(), dt);
    const double pitchTorque = cfg.pitch.Update(
        WrapPi(cfg.targetPitch - cfg.measuredEuler.Y()),
        cfg.measuredBodyRates.Y(), dt);
    const double yawManeuverError =
        std::abs(WrapPi(cfg.commandYaw - cfg.measuredEuler.Z()));
    if (yawManeuverError >= cfg.yawScheduleEnd)
      this->yawLargeSignalMode = true;
    else if (yawManeuverError <= cfg.yawScheduleStart &&
             std::abs(cfg.measuredBodyRates.Z()) <= 0.05)
      this->yawLargeSignalMode = false;
    const double yawLargeSignalBlend =
        this->yawLargeSignalMode ? 1.0 : 0.0;
    cfg.yaw.kp = (1.0 - yawLargeSignalBlend) * cfg.yaw.kp +
        yawLargeSignalBlend * cfg.yawLargeKp;
    cfg.yaw.kd = (1.0 - yawLargeSignalBlend) * cfg.yaw.kd +
        yawLargeSignalBlend * cfg.yawLargeKd;
    const double yawTorque = cfg.yaw.Update(
        WrapPi(cfg.targetYaw - cfg.measuredEuler.Z()),
        cfg.measuredBodyRates.Z(), dt);

    const gz::math::Vector3d requestedTorque(
        rollTorque, pitchTorque, yawTorque);
    const ActuatorCommand delayed = this->DelayedCommand(
        ActuatorCommand{simTime, omega, requestedTorque}, cfg);
    const double appliedOmega =
        this->UpdateMotorResponse(delayed.omega, cfg, dt);
    const auto torqueWorld =
        pose->Rot().RotateVector(delayed.torqueBody);
    this->link.AddWorldWrench(_ecm, gz::math::Vector3d::Zero, torqueWorld);
    this->PublishMotorSpeeds(appliedOmega);
    this->PublishStatus(_info, *pose, *linearVel, bodyRates, cfg,
        omega, appliedOmega, upperOmega, lowerOmega,
        requestedTorque, delayed.torqueBody, true, yawLargeSignalBlend);
  }

  private: struct ConfigSnapshot
  {
    bool enabled{false};
    double targetZ{0.8};
    double targetRoll{0.0};
    double targetPitch{0.0};
    double targetYaw{0.0};
    double commandRoll{0.0};
    double commandPitch{0.0};
    double commandYaw{0.0};
    double maxOmega{156.0};
    bool rotorInterferenceEnabled{true};
    double coaxialMaxThrustLoss{0.06};
    Pid height;
    Pid roll;
    Pid pitch;
    Pid yaw;
    double yawLargeKp{20.0};
    double yawLargeKd{3.0};
    double yawScheduleStart{0.02};
    double yawScheduleEnd{0.08};
    bool velocityControlEnabled{false};
    double targetVx{0.0};
    double targetVy{0.0};
    double velocityAccelX{0.0};
    double velocityAccelY{0.0};
    double velocityAccelLimit{2.2};
    bool positionControlEnabled{false};
    double targetX{0.0};
    double targetY{0.0};
    double positionVelocityLimit{2.5};
    gz::math::Vector3d measuredPosition{0.0, 0.0, 0.0};
    gz::math::Vector3d measuredLinearVelocity{0.0, 0.0, 0.0};
    gz::math::Vector3d measuredEuler{0.0, 0.0, 0.0};
    gz::math::Vector3d measuredBodyRates{0.0, 0.0, 0.0};
    bool nonidealitiesEnabled{false};
    double controlDelay{0.0};
    double motorTimeConstant{0.0};
    double motorRateLimit{0.0};
    double motorEffectiveness{1.0};
    double targetVz{0.0};
    double filteredBuoyancy{0.0};
    bool waterContact{false};
    double leftSubmerged{0.0};
    double rightSubmerged{0.0};
    double slammingForce{0.0};
    std::string landingState{"IDLE"};
    bool landingActive{false};
    double landingStateStart{0.0};
    double floatClearance{0.0};
    double floatSignedClearance{0.0};
    double landingPeakImpact{0.0};
    double landingImpactImpulse{0.0};
    double touchdownVz{0.0};
    double landingTargetX{0.0};
    double landingTargetY{0.0};
    double landingTargetYaw{0.0};
    double landingMissionHoverZ{0.0};
    bool landingStartedOnWater{false};
    std::string landingSurfaceMode{"water"};
    double landingSurfaceZ{0.0};
    double landingPlatformTopOffset{0.20};
    bool landingVehicleGeometryReady{false};
    bool landingSurfaceGeometryReady{false};
    double landingFloatBottomOffset{0.0};
    double landingWaterEquilibriumBodyOffset{0.0};
    double landingPlatformSafeHalfLength{0.0};
    double landingPlatformSafeHalfWidth{0.0};
    double landingContactMinClearance{0.0};
    double landingContactMaxClearance{0.0};
    bool landingPlatformAvailable{false};
    bool landingPlatformContact{false};
    double landingApproachSpeed{0.8};
    double landingCruiseSpeed{2.5};
    double landingDepartureHorizontalSpeedLimit{0.30};
    double landingNearHorizontalSpeedLimit{0.30};
    double landingMovingTargetCorrectionReserve{0.30};
    double landingApproachBrakingAccel{0.55};
    double landingDepartureTiltLimit{0.0872665};
    double landingApproachTiltLimit{0.174533};
    double landingNearTiltLimit{0.0872665};
    double landingWarningTilt{0.0872665};
    double landingAbortTilt{0.139626};
    double landingApproachAbortTilt{0.209440};
    double landingAbortPositionError{0.40};
    double landingNearMaxDescentSpeed{0.30};
    double landingGoAroundHeight{1.0};
    double landingContactLossGrace{0.08};
    double landingContactSubmergedFraction{0.02};
    bool landingMovingTargetEnabled{false};
    double landingTargetVx{0.0};
    double landingTargetVy{0.0};
    double landingTargetYawRate{0.0};
    bool landingTargetHealthy{true};
    double landingTargetStatusAge{0.0};
    double touchdownHorizontalError{0.0};
    double touchdownRelativeSpeed{0.0};
    double touchdownYawError{0.0};
    double dualContactDelay{0.0};
    std::string abortReason;
    std::string abortTriggerState;
    double abortMeasuredValue{0.0};
    double abortLimitValue{0.0};
    bool spoolDown{false};
    double spoolOmega{0.0};
  };

  private: struct ActuatorCommand
  {
    double simTime{0.0};
    double omega{0.0};
    gz::math::Vector3d torqueBody{0.0, 0.0, 0.0};
  };

  private: double Noise(double _stddev)
  {
    if (_stddev <= 0.0)
      return 0.0;
    return std::normal_distribution<double>(0.0, _stddev)(this->rng);
  }

  private: void ResetNoiseGenerator()
  {
    this->rng.seed(this->nonidealitiesSeed);
    this->attitudeBias = gz::math::Vector3d(
        this->Noise(this->attitudeBiasStd),
        this->Noise(this->attitudeBiasStd),
        this->Noise(this->attitudeBiasStd));
    this->gyroBias = gz::math::Vector3d(
        this->Noise(this->gyroBiasStd),
        this->Noise(this->gyroBiasStd),
        this->Noise(this->gyroBiasStd));
  }

  private: ActuatorCommand DelayedCommand(
      const ActuatorCommand &_requested, const ConfigSnapshot &_cfg)
  {
    if (!_cfg.nonidealitiesEnabled || _cfg.controlDelay <= 0.0)
    {
      this->commandQueue.clear();
      this->lastDelayedCommand = _requested;
      return _requested;
    }
    this->commandQueue.push_back(_requested);
    const double readyTime = _requested.simTime - _cfg.controlDelay;
    while (!this->commandQueue.empty() &&
           this->commandQueue.front().simTime <= readyTime)
    {
      this->lastDelayedCommand = this->commandQueue.front();
      this->commandQueue.pop_front();
    }
    return this->lastDelayedCommand;
  }

  private: double UpdateMotorResponse(double _target,
      const ConfigSnapshot &_cfg, double _dt)
  {
    if (!_cfg.nonidealitiesEnabled)
    {
      this->appliedMotorOmega = _target;
      return _target;
    }
    const double effectiveTarget = _target * _cfg.motorEffectiveness;
    double next = effectiveTarget;
    if (_cfg.motorTimeConstant > 0.0)
    {
      const double alpha = _dt / (_cfg.motorTimeConstant + _dt);
      next = this->appliedMotorOmega +
          alpha * (effectiveTarget - this->appliedMotorOmega);
    }
    if (_cfg.motorRateLimit > 0.0)
    {
      const double maxStep = _cfg.motorRateLimit * _dt;
      next = this->appliedMotorOmega + Clamp(
          next - this->appliedMotorOmega, -maxStep, maxStep);
    }
    this->appliedMotorOmega = std::max(0.0, next);
    return this->appliedMotorOmega;
  }

  private: void SetJointSpeeds(gz::sim::EntityComponentManager &_ecm,
      double _omega)
  {
    this->SetJointSpeed(_ecm, this->upperJoint.Entity(), _omega);
    this->SetJointSpeed(_ecm, this->lowerJoint.Entity(), -_omega);
  }

  private: void PublishMotorSpeeds(double _omega)
  {
    gz::msgs::Actuators command;
    command.add_velocity(_omega);
    command.add_velocity(_omega);
    this->motorCommandPub.Publish(command);
  }

  private: void SetJointSpeed(gz::sim::EntityComponentManager &_ecm,
      gz::sim::Entity _joint, double _speed)
  {
    if (_joint == gz::sim::kNullEntity)
      return;
    auto comp = _ecm.Component<gz::sim::components::JointVelocityCmd>(_joint);
    if (!comp)
    {
      _ecm.CreateComponent(_joint,
          gz::sim::components::JointVelocityCmd({std::vector<double>{_speed}}));
      return;
    }
    comp->Data() = std::vector<double>{_speed};
  }

  private: void SetLandingState(const std::string &_state, double _simTime)
  {
    this->landingState = _state;
    this->landingStateStart = _simTime;
    this->landingStableSince = -1.0;
  }

  private: void SetLandingAbort(const std::string &_reason,
      double _measuredValue, double _limitValue)
  {
    this->landingAbortReason = _reason;
    this->landingAbortTriggerState = this->landingState;
    this->landingAbortMeasuredValue = _measuredValue;
    this->landingAbortLimitValue = _limitValue;
  }

  private: void RestoreLandingLimits()
  {
    if (!this->landingLimitsSaved)
      return;
    this->positionVelocityLimit = this->landingSavedPositionVelocityLimit;
    this->velocityTiltLimit = this->landingSavedVelocityTiltLimit;
    this->landingLimitsSaved = false;
  }

  private: double LandingPositionLimit(double _limit) const
  {
    return this->landingSavedPositionVelocityLimit > 0.0 ?
        std::min(this->landingSavedPositionVelocityLimit, _limit) : _limit;
  }

  private: double LandingTrackingPositionLimit(
      double _absoluteLimit) const
  {
    double requestedLimit = _absoluteLimit;
    if (this->landingActive && this->landingMovingTargetEnabled &&
        !this->landingTargetTrackingFrozen)
    {
      const double targetSpeed = std::hypot(
          this->landingTargetVx, this->landingTargetVy);
      requestedLimit = std::max(
          requestedLimit,
          targetSpeed + this->landingMovingTargetCorrectionReserve);
    }
    return this->LandingPositionLimit(requestedLimit);
  }

  private: double LandingTiltLimit(double _limit) const
  {
    return this->landingSavedVelocityTiltLimit > 0.0 ?
        std::min(this->landingSavedVelocityTiltLimit, _limit) : _limit;
  }

  private: double LandingContactSupportZ() const
  {
    return this->landingSurfaceZ +
        (this->landingSurfaceMode == "platform" ?
         this->landingFloatBottomOffset :
         this->landingWaterEquilibriumBodyOffset);
  }

  private: double PlatformSafeHalfLength() const
  {
    return std::max(0.0, this->landingPlatformHalfLength -
        this->landingFloatFootprintHalfLength -
        this->landingPlatformEdgeMargin);
  }

  private: double PlatformSafeHalfWidth() const
  {
    return std::max(0.0, this->landingPlatformHalfWidth -
        this->landingFloatFootprintHalfWidth -
        this->landingPlatformEdgeMargin);
  }

  private: bool PlatformContact(
      const gz::math::Vector3d &_position) const
  {
    if (this->landingSurfaceMode != "platform" ||
        !this->landingPlatformAvailable)
      return false;
    const double dx = _position.X() - this->landingTargetX;
    const double dy = _position.Y() - this->landingTargetY;
    const double c = std::cos(this->landingTargetYaw);
    const double s = std::sin(this->landingTargetYaw);
    const double localX = c * dx + s * dy;
    const double localY = -s * dx + c * dy;
    const double clearance = _position.Z() -
        this->landingFloatBottomOffset - this->landingSurfaceZ;
    const double safeHalfLength = this->PlatformSafeHalfLength();
    const double safeHalfWidth = this->PlatformSafeHalfWidth();
    return this->landingVehicleGeometryReady &&
        this->landingSurfaceGeometryReady &&
        safeHalfLength > 0.0 && safeHalfWidth > 0.0 &&
        std::abs(localX) <= safeHalfLength &&
        std::abs(localY) <= safeHalfWidth &&
        clearance >= this->landingContactMinClearance &&
        clearance <= this->landingContactMaxClearance;
  }

  private: void UpdateLandingLocked(
      double _simTime, double _dt,
      const gz::math::Vector3d &_position,
      const gz::math::Vector3d &_linearVelocity,
      const gz::math::Vector3d &_euler,
      const gz::math::Vector3d &_bodyRates)
  {
    if (this->landingStartRequested)
    {
      this->landingStartRequested = false;
      this->landingLocalLandRequested = false;
      this->landingLocalLandMode = false;
      this->landingActive = true;
      this->landingAbortReason.clear();
      this->landingAbortTriggerState.clear();
      this->landingAbortMeasuredValue = 0.0;
      this->landingAbortLimitValue = 0.0;
      this->landingPeakImpact = 0.0;
      this->landingImpactImpulse = 0.0;
      this->landingTouchdownVz = 0.0;
      this->landingTouchdownHorizontalError = 0.0;
      this->landingTouchdownRelativeSpeed = 0.0;
      this->landingTouchdownYawError = 0.0;
      this->landingDualContactDelay = 0.0;
      this->landingFirstContactTime = -1.0;
      this->landingContactUnstableSince = -1.0;
      this->landingTargetTrackingFrozen = false;
      this->landingMissionStartTime = _simTime;
      this->landingTargetX = this->landingConfiguredTargetX;
      this->landingTargetY = this->landingConfiguredTargetY;
      this->landingTargetYaw = this->landingConfiguredTargetYaw;
      this->landingTargetVz = 0.0;
      this->landingSpoolOmega = 0.0;
      this->enabled = true;
      this->positionControlEnabled = true;
      this->velocityControlEnabled = false;
      this->height.integral = 0.0;
      this->positionX.integral = 0.0;
      this->positionY.integral = 0.0;
      this->velocityX.integral = 0.0;
      this->velocityY.integral = 0.0;
      if (!this->landingVehicleGeometryReady ||
          (this->landingSurfaceMode == "platform" &&
           !this->landingSurfaceGeometryReady))
      {
        this->SetLandingAbort("geometry unavailable", 0.0, 1.0);
        this->enabled = false;
        this->landingActive = false;
        this->SetLandingState("ABORTED", _simTime);
        return;
      }
      this->landingSavedPositionVelocityLimit =
          this->positionVelocityLimit;
      this->landingSavedVelocityTiltLimit = this->velocityTiltLimit;
      this->landingLimitsSaved = true;
      this->landingDepartureX = _position.X();
      this->landingDepartureY = _position.Y();
      this->landingDepartureYaw = _euler.Z();
      this->landingStartedOnWater = this->landingSurfaceMode == "platform" ?
          this->PlatformContact(_position) :
          (this->waterContact ||
           this->leftSubmerged >= this->landingContactSubmergedFraction ||
           this->rightSubmerged >= this->landingContactSubmergedFraction);
      const double distanceToLanding = std::hypot(
          this->landingTargetX - _position.X(),
          this->landingTargetY - _position.Y());
      const double configuredApproachZ =
          this->landingSurfaceZ + this->landingHighHoverZ;
      const double minimumTransitZ =
          this->landingSurfaceZ + this->landingFloatBottomOffset +
          this->landingFlareClearance +
          this->landingDepartureClearanceMargin;
      if (this->landingStartedOnWater)
        this->landingMissionHoverZ = configuredApproachZ;
      else if (distanceToLanding > this->landingPositionTolerance)
        this->landingMissionHoverZ =
            std::max(_position.Z(), minimumTransitZ);
      else
        this->landingMissionHoverZ = std::max(
            _position.Z(), this->LandingContactSupportZ());
      this->targetZ = this->landingMissionHoverZ;
      if (!this->landingStartedOnWater &&
          distanceToLanding <= this->landingPositionTolerance)
      {
        this->targetX = this->landingTargetX;
        this->targetY = this->landingTargetY;
        this->targetYaw = this->landingTargetYaw;
        this->SetLandingState("ALIGN", _simTime);
      }
      else
      {
        this->targetX = this->landingDepartureX;
        this->targetY = this->landingDepartureY;
        this->targetYaw = this->landingDepartureYaw;
        this->SetLandingState(
            this->landingMissionHoverZ > _position.Z() + 0.08 ?
            "CLIMB" : "STABILIZE", _simTime);
      }
    }
    if (!this->landingActive)
      return;

    if (this->landingSurfaceMode == "water")
    {
      this->landingPeakImpact =
          std::max(this->landingPeakImpact, this->waterSlammingForce);
      this->landingImpactImpulse +=
          std::max(0.0, this->waterSlammingForce) * _dt;
    }
    const bool departureHold =
        this->landingState == "CLIMB" ||
        this->landingState == "STABILIZE";
    if (!departureHold && this->landingState != "GO_AROUND" &&
        !this->landingTargetTrackingFrozen)
    {
      this->targetX = this->landingTargetX;
      this->targetY = this->landingTargetY;
      this->targetYaw = this->landingTargetYaw;
    }
    const double horizontalError = std::hypot(
        this->targetX - _position.X(), this->targetY - _position.Y());
    const double horizontalSpeed =
        std::hypot(_linearVelocity.X(), _linearVelocity.Y());
    const double relativeHorizontalSpeed = std::hypot(
        _linearVelocity.X() - this->landingTargetVx,
        _linearVelocity.Y() - this->landingTargetVy);
    const double maxTilt = std::max(
        std::abs(_euler.X()), std::abs(_euler.Y()));
    const double maxTiltRate = std::max(
        std::abs(_bodyRates.X()), std::abs(_bodyRates.Y()));
    const double yawError =
        std::abs(WrapPi(this->landingTargetYaw - _euler.Z()));
    const double relativeYawRate =
        std::abs(_bodyRates.Z() - this->landingTargetYawRate);
    const double targetSpeed =
        std::hypot(this->landingTargetVx, this->landingTargetVy);
    const bool targetStatusRequired =
        this->landingMovingTargetEnabled ||
        this->landingSurfaceMode == "platform";
    this->landingTargetStatusAge =
        !targetStatusRequired ? 0.0 :
        (this->landingTargetStatusSimTime >= 0.0 ?
         std::max(0.0, _simTime - this->landingTargetStatusSimTime) :
         this->landingTargetStatusTimeout + 1.0);
    this->landingTargetHealthy = !targetStatusRequired ||
        (this->landingTargetStatusValid &&
         this->landingTargetStatusAge <= this->landingTargetStatusTimeout &&
         (!this->landingMovingTargetEnabled ||
          targetSpeed <= this->landingTargetSpeedLimit) &&
         (this->landingSurfaceMode != "platform" ||
          this->landingPlatformAvailable));
    const double clearance =
        _position.Z() - this->landingFloatBottomOffset -
        this->landingSurfaceZ;

    if (this->landingLocalLandRequested)
    {
      this->landingLocalLandRequested = false;
      this->landingLocalLandMode = true;
      this->landingAbortReason.clear();
      this->landingAbortTriggerState.clear();
      this->landingAbortMeasuredValue = 0.0;
      this->landingAbortLimitValue = 0.0;
      this->landingMovingTargetEnabled = false;
      this->landingTargetTrackingFrozen = true;
      this->landingTargetX = _position.X();
      this->landingTargetY = _position.Y();
      this->landingTargetYaw = _euler.Z();
      this->landingTargetVx = 0.0;
      this->landingTargetVy = 0.0;
      this->landingTargetYawRate = 0.0;
      this->targetX = this->landingTargetX;
      this->targetY = this->landingTargetY;
      this->targetYaw = this->landingTargetYaw;
      this->landingMissionHoverZ = std::max(
          _position.Z(), this->LandingContactSupportZ());
      this->targetZ = this->landingMissionHoverZ;
      this->positionX.integral = 0.0;
      this->positionY.integral = 0.0;
      this->velocityX.integral = 0.0;
      this->velocityY.integral = 0.0;
      this->SetLandingState(
          clearance <= this->landingFlareClearance +
              this->landingFlareTransitionMargin ?
          "NEAR_WATER" : "ALIGN", _simTime);
    }
    const bool beforeContact =
        this->landingState == "CLIMB" ||
        this->landingState == "STABILIZE" ||
        this->landingState == "APPROACH" ||
        this->landingState == "ALIGN" ||
        this->landingState == "HIGH_HOVER" ||
        this->landingState == "SLOW_DESCENT" ||
        this->landingState == "NEAR_WATER";
    if (beforeContact && this->landingState != "GO_AROUND" &&
        this->landingLocalLandMode)
    {
      const bool localRecoveryNeeded =
          maxTilt > this->landingAbortTilt ||
          horizontalError > this->landingAbortPositionError ||
          (this->landingState == "NEAR_WATER" &&
           _simTime - this->landingStateStart >
               this->landingNearOverspeedGrace &&
           _linearVelocity.Z() < -this->landingNearMaxDescentSpeed);
      if (localRecoveryNeeded)
      {
        this->landingMissionHoverZ = std::max(
            _position.Z(), this->LandingContactSupportZ());
        this->targetZ = this->landingMissionHoverZ;
        this->landingTargetVz = 0.0;
        this->SetLandingState("ALIGN", _simTime);
      }
    }
    else if (beforeContact && this->landingState != "GO_AROUND")
    {
      const bool approachPhase =
          this->landingState == "CLIMB" ||
          this->landingState == "STABILIZE" ||
          this->landingState == "APPROACH" ||
          this->landingState == "ALIGN";
      const double abortTilt = approachPhase ?
          this->landingApproachAbortTilt : this->landingAbortTilt;
      if (targetStatusRequired &&
          _simTime - this->landingMissionStartTime >
              this->landingTargetStatusTimeout &&
          !this->landingTargetHealthy)
      {
        if (this->landingSurfaceMode == "platform" &&
            !this->landingPlatformAvailable)
          this->SetLandingAbort("platform unavailable", 0.0, 1.0);
        else if (targetSpeed > this->landingTargetSpeedLimit)
          this->SetLandingAbort("target speed limit", targetSpeed,
              this->landingTargetSpeedLimit);
        else
          this->SetLandingAbort("target status lost",
              this->landingTargetStatusAge,
              this->landingTargetStatusTimeout);
        if (this->landingStartedOnWater &&
            (this->landingState == "CLIMB" ||
             this->landingState == "STABILIZE"))
        {
          this->enabled = false;
          this->landingActive = false;
          this->RestoreLandingLimits();
          this->SetLandingState("ABORTED", _simTime);
        }
        else
        {
          this->SetLandingState("GO_AROUND", _simTime);
        }
      }
      else if (maxTilt > abortTilt)
      {
        this->SetLandingAbort("attitude limit", maxTilt, abortTilt);
        this->SetLandingState("GO_AROUND", _simTime);
      }
      else if (!approachPhase &&
               horizontalError > this->landingAbortPositionError)
      {
        this->SetLandingAbort("position error", horizontalError,
            this->landingAbortPositionError);
        this->SetLandingState("GO_AROUND", _simTime);
      }
      else if (this->landingState == "NEAR_WATER" &&
               _simTime - this->landingStateStart >
                   this->landingNearOverspeedGrace &&
               _linearVelocity.Z() < -this->landingNearMaxDescentSpeed)
      {
        this->SetLandingAbort("descent speed",
            std::abs(_linearVelocity.Z()),
            this->landingNearMaxDescentSpeed);
        this->SetLandingState("GO_AROUND", _simTime);
      }
    }

    this->positionControlEnabled = true;
    this->velocityControlEnabled = false;
    if (this->landingSurfaceMode == "platform" &&
        this->PlatformContact(_position) &&
        horizontalError <= this->landingPositionTolerance &&
        relativeHorizontalSpeed <=
            this->landingHoverRelativeSpeedTolerance &&
        (this->landingState == "ALIGN" ||
         this->landingState == "HIGH_HOVER" ||
         this->landingState == "SLOW_DESCENT"))
    {
      this->targetZ = std::max(
          this->LandingContactSupportZ(), _position.Z());
      this->landingTargetVz = 0.0;
      this->SetLandingState("NEAR_WATER", _simTime);
    }
    if (this->landingState == "CLIMB" ||
        this->landingState == "STABILIZE")
    {
      this->targetX = this->landingDepartureX;
      this->targetY = this->landingDepartureY;
      this->targetYaw = this->landingDepartureYaw;
      this->targetZ = this->landingMissionHoverZ;
      this->landingTargetVz = 0.0;
      this->positionVelocityLimit = this->LandingPositionLimit(
          this->landingDepartureHorizontalSpeedLimit);
      this->velocityTiltLimit = this->LandingTiltLimit(
          this->landingDepartureTiltLimit);
      const bool safeForTranslationalApproach =
          this->landingState == "CLIMB" &&
          this->landingStartedOnWater &&
          clearance >= this->landingFlareClearance +
              this->landingDepartureClearanceMargin &&
          _linearVelocity.Z() >=
              -this->landingApproachVerticalSpeedTolerance;
      const bool clearAndStable =
          std::abs(_position.Z() - this->targetZ) <=
              this->landingHeightTolerance &&
          std::abs(_linearVelocity.Z()) <=
              this->landingApproachVerticalSpeedTolerance &&
          horizontalSpeed <=
              this->landingDepartureHorizontalSpeedTolerance &&
          maxTilt <= this->landingDepartureTiltLimit;
      if (safeForTranslationalApproach)
      {
        this->targetX = this->landingTargetX;
        this->targetY = this->landingTargetY;
        this->targetYaw = this->landingTargetYaw;
        this->SetLandingState("APPROACH", _simTime);
      }
      else if (clearAndStable)
      {
        if (this->landingStableSince < 0.0)
          this->landingStableSince = _simTime;
        else if (_simTime - this->landingStableSince >=
                 this->landingDepartureStableTime)
        {
          this->targetX = this->landingTargetX;
          this->targetY = this->landingTargetY;
          this->targetYaw = this->landingTargetYaw;
          this->SetLandingState("APPROACH", _simTime);
        }
      }
      else
        this->landingStableSince = -1.0;
    }
    else if (this->landingState == "APPROACH")
    {
      this->targetZ = this->landingMissionHoverZ;
      this->targetYaw = this->landingTargetYaw;
      this->landingTargetVz = 0.0;
      const double stoppingSpeed = std::sqrt(
          2.0 * this->landingApproachBrakingAccel * std::max(
              0.0, horizontalError - this->landingPositionTolerance));
      const double adaptiveApproachLimit = std::min(
          this->landingCruiseSpeed,
          std::max(this->landingApproachSpeed, stoppingSpeed));
      this->positionVelocityLimit =
          this->LandingTrackingPositionLimit(adaptiveApproachLimit);
      this->velocityTiltLimit = this->LandingTiltLimit(
          this->landingApproachTiltLimit);
      const bool arrived =
          horizontalError <= this->landingPositionTolerance &&
          relativeHorizontalSpeed <=
              this->landingApproachRelativeSpeedTolerance &&
          std::abs(_position.Z() - this->targetZ) <=
              this->landingHeightTolerance &&
          std::abs(_linearVelocity.Z()) <=
              this->landingApproachVerticalSpeedTolerance;
      if (arrived)
        this->SetLandingState("ALIGN", _simTime);
    }
    else if (this->landingState == "ALIGN")
    {
      this->targetZ = this->landingMissionHoverZ;
      this->targetYaw = this->landingTargetYaw;
      this->landingTargetVz = 0.0;
      this->positionVelocityLimit =
          this->LandingTrackingPositionLimit(
              this->landingNearHorizontalSpeedLimit);
      this->velocityTiltLimit = this->LandingTiltLimit(
          this->landingNearTiltLimit);
      const bool aligned =
          horizontalError <= this->landingPositionTolerance &&
          relativeHorizontalSpeed <=
              this->landingAlignRelativeSpeedTolerance &&
          yawError <= this->landingYawTolerance &&
          relativeYawRate <= this->landingYawRateTolerance &&
          std::abs(_position.Z() - this->targetZ) <=
              this->landingHeightTolerance &&
          std::abs(_linearVelocity.Z()) <=
              this->landingPrecisionVerticalSpeedTolerance;
      if (aligned)
      {
        if (this->landingStableSince < 0.0)
          this->landingStableSince = _simTime;
        else if (_simTime - this->landingStableSince >=
                 this->landingAlignStableTime)
          this->SetLandingState("HIGH_HOVER", _simTime);
      }
      else
        this->landingStableSince = -1.0;
    }
    else if (this->landingState == "HIGH_HOVER")
    {
      this->targetZ = this->landingMissionHoverZ;
      this->targetYaw = this->landingTargetYaw;
      this->landingTargetVz = 0.0;
      this->positionVelocityLimit =
          this->LandingTrackingPositionLimit(
              this->landingNearHorizontalSpeedLimit);
      this->velocityTiltLimit = this->LandingTiltLimit(
          this->landingNearTiltLimit);
      const bool stable =
          std::abs(_position.Z() - this->targetZ) <=
              this->landingHeightTolerance &&
          std::abs(_linearVelocity.Z()) <=
              this->landingPrecisionVerticalSpeedTolerance &&
          horizontalError <= this->landingPositionTolerance &&
          relativeHorizontalSpeed <=
              this->landingHoverRelativeSpeedTolerance &&
          yawError <= this->landingYawTolerance &&
          maxTilt <= this->landingNearTiltLimit;
      if (stable)
      {
        if (this->landingStableSince < 0.0)
          this->landingStableSince = _simTime;
        else if (_simTime - this->landingStableSince >=
                 this->landingHoverStableTime)
        {
          this->SetLandingState(
              clearance <= this->landingFlareClearance +
                  this->landingFlareTransitionMargin ?
              "NEAR_WATER" : "SLOW_DESCENT", _simTime);
        }
      }
      else
        this->landingStableSince = -1.0;
    }
    else if (this->landingState == "SLOW_DESCENT")
    {
      this->positionVelocityLimit =
          this->LandingTrackingPositionLimit(
              this->landingNearHorizontalSpeedLimit);
      this->velocityTiltLimit =
          this->LandingTiltLimit(this->landingNearTiltLimit);
      this->landingTargetVz = -this->landingDescentRate;
      this->targetZ = std::max(
          this->landingSurfaceZ + this->landingFloatBottomOffset +
              this->landingFlareClearance,
          this->targetZ - this->landingDescentRate * _dt);
      if (clearance <= this->landingFlareClearance +
          this->landingFlareTransitionMargin)
        this->SetLandingState("NEAR_WATER", _simTime);
    }
    else if (this->landingState == "NEAR_WATER")
    {
      this->velocityTiltLimit =
          std::min(this->velocityTiltLimit, this->landingNearTiltLimit);
      this->positionVelocityLimit =
          this->LandingTrackingPositionLimit(
              this->landingNearHorizontalSpeedLimit);
      this->landingTargetVz = -this->landingFlareRate;
      this->targetZ = std::max(
          this->LandingContactSupportZ(),
          this->targetZ - this->landingFlareRate * _dt);
      const bool platformContact = this->PlatformContact(_position);
      const bool bothContact = this->landingSurfaceMode == "platform" ?
          (platformContact &&
           horizontalError <= this->landingPositionTolerance &&
           relativeHorizontalSpeed <=
               this->landingHoverRelativeSpeedTolerance) :
          (this->leftSubmerged >= this->landingContactSubmergedFraction &&
           this->rightSubmerged >= this->landingContactSubmergedFraction);
      const bool eitherContact = this->landingSurfaceMode == "platform" ?
          platformContact :
          (this->leftSubmerged >= this->landingContactSubmergedFraction ||
           this->rightSubmerged >= this->landingContactSubmergedFraction);
      if (eitherContact && this->landingFirstContactTime < 0.0)
        this->landingFirstContactTime = _simTime;
      if (bothContact && std::abs(_linearVelocity.Z()) <=
          this->landingTouchdownMaxVz)
      {
        this->landingTouchdownVz = _linearVelocity.Z();
        this->landingTouchdownHorizontalError = horizontalError;
        this->landingTouchdownRelativeSpeed = std::hypot(
            _linearVelocity.X() - this->landingTargetVx,
            _linearVelocity.Y() - this->landingTargetVy);
        this->landingTouchdownYawError = yawError;
        this->landingDualContactDelay =
            this->landingFirstContactTime >= 0.0 ?
            std::max(0.0, _simTime - this->landingFirstContactTime) : 0.0;
        this->landingTargetTrackingFrozen =
            !this->landingMovingTargetEnabled;
        if (this->landingTargetTrackingFrozen)
        {
          this->landingTargetVx = 0.0;
          this->landingTargetVy = 0.0;
          this->landingTargetYawRate = 0.0;
        }
        this->targetZ = this->LandingContactSupportZ();
        this->SetLandingState("CONTACT_CONFIRM", _simTime);
      }
    }
    else if (this->landingState == "CONTACT_CONFIRM")
    {
      this->landingTargetVz = 0.0;
      this->targetZ = this->LandingContactSupportZ();
      const bool stableSurfaceContact =
          this->landingSurfaceMode == "platform" ?
          (this->PlatformContact(_position) &&
           relativeHorizontalSpeed <=
               this->landingHoverRelativeSpeedTolerance) :
          (this->leftSubmerged >= this->landingContactSubmergedFraction &&
           this->rightSubmerged >= this->landingContactSubmergedFraction);
      const bool stableContact = stableSurfaceContact &&
          std::abs(_linearVelocity.Z()) <= this->landingTouchdownMaxVz &&
          maxTilt <= this->landingWarningTilt &&
          maxTiltRate <= this->landingContactTiltRateLimit;
      if (!stableContact)
      {
        this->landingStableSince = -1.0;
        if (this->landingContactUnstableSince < 0.0)
          this->landingContactUnstableSince = _simTime;
        else if (_simTime - this->landingContactUnstableSince >=
                 this->landingContactLossGrace)
        {
          this->landingContactUnstableSince = -1.0;
          this->landingTargetTrackingFrozen = false;
          this->SetLandingState("NEAR_WATER", _simTime);
        }
      }
      else
      {
        this->landingContactUnstableSince = -1.0;
        if (this->landingStableSince < 0.0)
          this->landingStableSince = _simTime;
        else if (_simTime - this->landingStableSince >=
                 this->landingContactConfirmTime)
          this->SetLandingState("SETTLING", _simTime);
      }
    }
    else if (this->landingState == "SETTLING")
    {
      this->landingTargetVz = 0.0;
      this->targetZ = this->LandingContactSupportZ();
      this->height.integral = 0.0;
      const bool stableSurfaceSupport =
          this->landingSurfaceMode == "platform" ?
          this->PlatformContact(_position) :
          (this->leftSubmerged >= this->landingContactSubmergedFraction &&
           this->rightSubmerged >= this->landingContactSubmergedFraction);
      const bool stableFloat =
          stableSurfaceSupport &&
          maxTilt <= this->landingWarningTilt &&
          maxTiltRate <= this->landingSettlingTiltRateLimit &&
          std::abs(_linearVelocity.Z()) <=
              this->landingSettlingVerticalSpeedLimit;
      if (stableFloat)
      {
        if (this->landingStableSince < 0.0)
          this->landingStableSince = _simTime;
        else if (_simTime - this->landingStableSince >=
                 this->landingSettlingTime)
        {
          this->landingSpoolStartOmega = this->appliedMotorOmega;
          this->landingSpoolOmega = this->appliedMotorOmega;
          this->SetLandingState("SPOOL_DOWN", _simTime);
        }
      }
      else
      {
        this->landingStableSince = -1.0;
        if (!stableSurfaceSupport)
        {
          if (this->landingContactUnstableSince < 0.0)
            this->landingContactUnstableSince = _simTime;
          else if (_simTime - this->landingContactUnstableSince >=
                   this->landingContactLossGrace)
          {
            this->landingContactUnstableSince = -1.0;
            this->landingTargetTrackingFrozen = false;
            this->SetLandingState("NEAR_WATER", _simTime);
          }
        }
        else
          this->landingContactUnstableSince = -1.0;
      }
    }
    else if (this->landingState == "SPOOL_DOWN")
    {
      this->landingTargetVz = 0.0;
      this->height.integral = 0.0;
      this->positionX.integral = 0.0;
      this->positionY.integral = 0.0;
      const double progress = Clamp(
          (_simTime - this->landingStateStart) /
              std::max(this->landingSpoolDownTime, 0.1),
          0.0, 1.0);
      this->landingSpoolOmega =
          (1.0 - progress) * this->landingSpoolStartOmega;
      if (progress >= 1.0)
      {
        this->enabled = false;
        this->landingActive = false;
        this->landingTargetVz = 0.0;
        this->RestoreLandingLimits();
        this->SetLandingState("LANDED", _simTime);
      }
    }
    else if (this->landingState == "GO_AROUND")
    {
      this->landingTargetVz = 0.0;
      this->targetZ = std::max(
          this->landingSurfaceZ + this->landingGoAroundHeight,
          _position.Z());
      const bool recovered =
          _position.Z() >= this->targetZ -
              this->landingGoAroundHeightTolerance &&
          std::abs(_linearVelocity.Z()) <=
              this->landingGoAroundVerticalSpeedTolerance &&
          maxTilt <= this->landingGoAroundTiltTolerance;
      if (recovered)
      {
        this->landingActive = false;
        this->RestoreLandingLimits();
        this->SetLandingState("ABORTED", _simTime);
      }
    }
  }

  private: void OnLandingTargetStatus(const gz::msgs::StringMsg &_msg)
  {
    const auto text = _msg.data();
    std::lock_guard<std::mutex> lock(this->mutex);
    this->landingTargetStatusValid =
        BoolField(text, "valid", this->landingTargetStatusValid);
    this->landingTargetStatusSimTime = NumberField(
        text, "sim_time_s", this->landingTargetStatusSimTime);
    const auto reportedSurfaceMode = StringField(
        text, "surface_mode", "water");
    this->landingPlatformAvailable =
        BoolField(text, "platform_available", false) &&
        StringField(text, "platform_mode_version", "") == "solid_deck_v1";
    this->landingSurfaceGeometryReady =
        StringField(text, "surface_geometry_version", "") ==
        "solid_deck_geometry_v1";
    this->landingPlatformHalfLength = std::max(0.0, NumberField(text,
        "platform_half_length_m", this->landingPlatformHalfLength));
    this->landingPlatformHalfWidth = std::max(0.0, NumberField(text,
        "platform_half_width_m", this->landingPlatformHalfWidth));
    this->landingPlatformEdgeMargin = std::max(0.0, NumberField(text,
        "platform_edge_margin_m", this->landingPlatformEdgeMargin));
    this->landingContactMinClearance = NumberField(text,
        "contact_min_clearance_m", this->landingContactMinClearance);
    this->landingContactMaxClearance = NumberField(text,
        "contact_max_clearance_m", this->landingContactMaxClearance);
    if (reportedSurfaceMode == this->landingSurfaceMode)
    {
      this->landingSurfaceZ = Clamp(NumberField(
          text, "surface_z_m", this->landingSurfaceZ), -2.0, 5.0);
    }
    if (!this->landingMovingTargetEnabled ||
        this->landingTargetTrackingFrozen)
      return;
    this->landingTargetX = Clamp(NumberField(
        text, "x_m", this->landingTargetX), -100.0, 100.0);
    this->landingTargetY = Clamp(NumberField(
        text, "y_m", this->landingTargetY), -100.0, 100.0);
    this->landingTargetYaw = WrapPi(NumberField(
        text, "yaw_rad", this->landingTargetYaw));
    this->landingTargetVx = Clamp(NumberField(
        text, "vx_m_s", this->landingTargetVx), -2.0, 2.0);
    this->landingTargetVy = Clamp(NumberField(
        text, "vy_m_s", this->landingTargetVy), -2.0, 2.0);
    this->landingTargetYawRate = Clamp(NumberField(
        text, "yaw_rate_rad_s", this->landingTargetYawRate),
        -1.0, 1.0);
  }

  private: void OnWaterStatus(const gz::msgs::StringMsg &_msg)
  {
    const auto text = _msg.data();
    std::lock_guard<std::mutex> lock(this->mutex);
    this->waterContact = BoolField(
        text, "water_contact", this->waterContact);
    this->waterBuoyancy = std::max(
        0.0, NumberField(text, "buoyancy_n", this->waterBuoyancy));
    this->waterSlammingForce = std::max(
        0.0, NumberField(text, "slamming_force_n",
                         this->waterSlammingForce));
    this->leftSubmerged = Clamp(NumberField(
        text, "left_float_submerged_fraction", this->leftSubmerged),
        0.0, 1.0);
    this->rightSubmerged = Clamp(NumberField(
        text, "right_float_submerged_fraction", this->rightSubmerged),
        0.0, 1.0);
    this->landingVehicleGeometryReady =
        StringField(text, "vehicle_geometry_version", "") ==
        "float_geometry_v1";
    this->landingFloatBottomOffset = std::max(0.0, NumberField(text,
        "float_bottom_offset_m", this->landingFloatBottomOffset));
    this->landingFloatFootprintHalfLength = std::max(0.0, NumberField(text,
        "float_footprint_half_length_m",
        this->landingFloatFootprintHalfLength));
    this->landingFloatFootprintHalfWidth = std::max(0.0, NumberField(text,
        "float_footprint_half_width_m",
        this->landingFloatFootprintHalfWidth));
    this->landingWaterEquilibriumBodyOffset = std::max(0.0, NumberField(text,
        "water_equilibrium_body_offset_m",
        this->landingWaterEquilibriumBodyOffset));
  }

  private: void OnConfig(const gz::msgs::StringMsg &_msg)
  {
    const auto text = _msg.data();
    std::lock_guard<std::mutex> lock(this->mutex);
    const bool resetIntegrators = BoolField(text, "reset_integrators", false);
    if (resetIntegrators)
    {
      this->height.integral = 0.0;
      this->roll.integral = 0.0;
      this->pitch.integral = 0.0;
      this->yaw.integral = 0.0;
      this->velocityX.integral = 0.0;
      this->velocityY.integral = 0.0;
      this->positionX.integral = 0.0;
      this->positionY.integral = 0.0;
    }
    this->enabled = BoolField(text, "enabled", this->enabled);
    if (BoolField(text, "landing_start", false))
      this->landingStartRequested = true;
    if (BoolField(text, "landing_local_land", false))
      this->landingLocalLandRequested = true;
    if (!this->enabled && this->landingActive &&
        !this->landingStartRequested)
    {
      this->landingActive = false;
      this->landingState = "IDLE";
      this->landingTargetVz = 0.0;
      this->RestoreLandingLimits();
    }
    this->landingHighHoverZ = Clamp(NumberField(text,
        "landing_high_hover_z_m", this->landingHighHoverZ), 0.8, 5.0);
    const auto requestedSurfaceMode = StringField(
        text, "landing_surface_mode", this->landingSurfaceMode);
    this->landingSurfaceMode = requestedSurfaceMode == "platform" ?
        "platform" : "water";
    this->landingConfiguredTargetX = Clamp(NumberField(text,
        "landing_target_x_m", this->landingConfiguredTargetX),
        -100.0, 100.0);
    this->landingConfiguredTargetY = Clamp(NumberField(text,
        "landing_target_y_m", this->landingConfiguredTargetY),
        -100.0, 100.0);
    this->landingConfiguredTargetYaw = WrapPi(NumberField(text,
        "landing_target_yaw_rad", this->landingConfiguredTargetYaw));
    if (!this->landingActive)
    {
      this->landingTargetX = this->landingConfiguredTargetX;
      this->landingTargetY = this->landingConfiguredTargetY;
      this->landingTargetYaw = this->landingConfiguredTargetYaw;
    }
    this->landingMovingTargetEnabled = BoolField(text,
        "landing_moving_target_enabled", this->landingMovingTargetEnabled);
    this->landingTargetVx = Clamp(NumberField(text,
        "landing_target_vx_m_s", this->landingTargetVx), -2.0, 2.0);
    this->landingTargetVy = Clamp(NumberField(text,
        "landing_target_vy_m_s", this->landingTargetVy), -2.0, 2.0);
    this->landingTargetYawRate = Clamp(NumberField(text,
        "landing_target_yaw_rate_rad_s", this->landingTargetYawRate),
        -1.0, 1.0);
    if (!this->landingMovingTargetEnabled)
    {
      this->landingTargetVx = 0.0;
      this->landingTargetVy = 0.0;
      this->landingTargetYawRate = 0.0;
    }
    this->landingTargetStatusTimeout = Clamp(NumberField(text,
        "landing_target_status_timeout_s",
        this->landingTargetStatusTimeout), 0.05, 2.0);
    this->landingTargetSpeedLimit = Clamp(NumberField(text,
        "landing_target_speed_limit_m_s",
        this->landingTargetSpeedLimit), 0.05, 2.0);
    this->landingApproachSpeed = Clamp(NumberField(text,
        "landing_approach_speed_m_s", this->landingApproachSpeed),
        0.10, 2.0);
    this->landingCruiseSpeed = std::max(
        this->landingApproachSpeed,
        Clamp(NumberField(text, "landing_cruise_speed_m_s",
            this->landingCruiseSpeed), 0.5, 3.0));
    this->landingPositionTolerance = Clamp(NumberField(text,
        "landing_position_tolerance_m", this->landingPositionTolerance),
        0.05, 0.50);
    this->landingYawTolerance = Clamp(NumberField(text,
        "landing_yaw_tolerance_rad", this->landingYawTolerance),
        0.0174533, 0.349066);
    this->landingDescentRate = Clamp(NumberField(text,
        "landing_descent_rate_m_s", this->landingDescentRate), 0.05, 0.8);
    this->landingFlareClearance = Clamp(NumberField(text,
        "landing_flare_clearance_m", this->landingFlareClearance), 0.15, 1.0);
    this->landingFlareRate = Clamp(NumberField(text,
        "landing_flare_rate_m_s", this->landingFlareRate), 0.03, 0.3);
    this->landingTouchdownMaxVz = Clamp(NumberField(text,
        "landing_touchdown_max_vz_m_s", this->landingTouchdownMaxVz),
        0.05, 0.5);
    this->landingContactConfirmTime = Clamp(NumberField(text,
        "landing_contact_confirm_s", this->landingContactConfirmTime),
        0.1, 2.0);
    this->landingSpoolDownTime = Clamp(NumberField(text,
        "landing_spool_down_s", this->landingSpoolDownTime), 0.3, 5.0);
    this->landingDepartureHorizontalSpeedLimit = Clamp(NumberField(text,
        "landing_departure_horizontal_speed_limit_m_s",
        this->landingDepartureHorizontalSpeedLimit), 0.05, 1.5);
    this->landingDepartureClearanceMargin = Clamp(NumberField(text,
        "landing_departure_clearance_margin_m",
        this->landingDepartureClearanceMargin), 0.02, 0.5);
    this->landingNearHorizontalSpeedLimit = Clamp(NumberField(text,
        "landing_near_horizontal_speed_limit_m_s",
        this->landingNearHorizontalSpeedLimit), 0.05, 1.5);
    this->landingMovingTargetCorrectionReserve = Clamp(NumberField(text,
        "landing_moving_target_correction_reserve_m_s",
        this->landingMovingTargetCorrectionReserve), 0.05, 1.5);
    this->landingApproachBrakingAccel = Clamp(NumberField(text,
        "landing_approach_braking_accel_m_s2",
        this->landingApproachBrakingAccel), 0.1, 3.0);
    this->landingAbortPositionError = Clamp(NumberField(text,
        "landing_abort_position_error_m",
        this->landingAbortPositionError), 0.1, 2.0);
    this->landingNearMaxDescentSpeed = Clamp(NumberField(text,
        "landing_near_max_descent_speed_m_s",
        this->landingNearMaxDescentSpeed), 0.1, 1.0);
    this->landingGoAroundHeight = Clamp(NumberField(text,
        "landing_go_around_height_m", this->landingGoAroundHeight), 0.3, 3.0);
    this->landingDepartureStableTime = Clamp(NumberField(text,
        "landing_departure_stable_time_s",
        this->landingDepartureStableTime), 0.1, 3.0);
    this->landingAlignStableTime = Clamp(NumberField(text,
        "landing_align_stable_time_s", this->landingAlignStableTime), 0.1, 3.0);
    this->landingHoverStableTime = Clamp(NumberField(text,
        "landing_hover_stable_time_s", this->landingHoverStableTime), 0.1, 5.0);
    this->landingApproachRelativeSpeedTolerance = Clamp(NumberField(text,
        "landing_approach_relative_speed_tolerance_m_s",
        this->landingApproachRelativeSpeedTolerance), 0.03, 0.8);
    this->landingAlignRelativeSpeedTolerance = Clamp(NumberField(text,
        "landing_align_relative_speed_tolerance_m_s",
        this->landingAlignRelativeSpeedTolerance), 0.03, 0.5);
    this->landingHoverRelativeSpeedTolerance = Clamp(NumberField(text,
        "landing_hover_relative_speed_tolerance_m_s",
        this->landingHoverRelativeSpeedTolerance), 0.03, 0.5);
    this->landingDepartureHorizontalSpeedTolerance = Clamp(NumberField(text,
        "landing_departure_horizontal_speed_tolerance_m_s",
        this->landingDepartureHorizontalSpeedTolerance), 0.03, 0.5);
    this->landingHeightTolerance = Clamp(NumberField(text,
        "landing_height_tolerance_m", this->landingHeightTolerance), 0.05, 0.5);
    this->landingApproachVerticalSpeedTolerance = Clamp(NumberField(text,
        "landing_approach_vertical_speed_tolerance_m_s",
        this->landingApproachVerticalSpeedTolerance), 0.03, 0.5);
    this->landingPrecisionVerticalSpeedTolerance = Clamp(NumberField(text,
        "landing_precision_vertical_speed_tolerance_m_s",
        this->landingPrecisionVerticalSpeedTolerance), 0.02, 0.3);
    this->landingNearOverspeedGrace = Clamp(NumberField(text,
        "landing_near_overspeed_grace_s",
        this->landingNearOverspeedGrace), 0.0, 3.0);
    this->landingContactSubmergedFraction = Clamp(NumberField(text,
        "landing_contact_submerged_fraction",
        this->landingContactSubmergedFraction), 0.005, 0.3);
    this->landingSettlingVerticalSpeedLimit = Clamp(NumberField(text,
        "landing_settling_vertical_speed_limit_m_s",
        this->landingSettlingVerticalSpeedLimit), 0.02, 0.3);
    this->landingSettlingTime = Clamp(NumberField(text,
        "landing_settling_time_s", this->landingSettlingTime), 0.1, 3.0);
    this->landingContactLossGrace = Clamp(NumberField(text,
        "landing_contact_loss_grace_s",
        this->landingContactLossGrace), 0.0, 0.5);
    this->landingGoAroundHeightTolerance = Clamp(NumberField(text,
        "landing_go_around_height_tolerance_m",
        this->landingGoAroundHeightTolerance), 0.05, 0.5);
    this->landingGoAroundVerticalSpeedTolerance = Clamp(NumberField(text,
        "landing_go_around_vertical_speed_tolerance_m_s",
        this->landingGoAroundVerticalSpeedTolerance), 0.03, 0.5);
    this->landingFlareTransitionMargin = Clamp(NumberField(text,
        "landing_flare_transition_margin_m",
        this->landingFlareTransitionMargin), 0.0, 0.15);
    this->landingDepartureTiltLimit = Clamp(NumberField(text,
        "landing_departure_tilt_limit_rad",
        this->landingDepartureTiltLimit), 0.0174533, 0.349066);
    this->landingApproachTiltLimit = Clamp(NumberField(text,
        "landing_approach_tilt_limit_rad",
        this->landingApproachTiltLimit), 0.0523599, 0.523599);
    this->landingNearTiltLimit = Clamp(NumberField(text,
        "landing_near_tilt_limit_rad",
        this->landingNearTiltLimit), 0.0174533, 0.349066);
    this->landingWarningTilt = Clamp(NumberField(text,
        "landing_warning_tilt_rad", this->landingWarningTilt),
        0.0174533, 0.349066);
    this->landingAbortTilt = std::max(this->landingWarningTilt,
        Clamp(NumberField(text, "landing_abort_tilt_rad",
            this->landingAbortTilt), 0.0523599, 0.523599));
    this->landingApproachAbortTilt = std::max(this->landingAbortTilt,
        Clamp(NumberField(text, "landing_approach_abort_tilt_rad",
            this->landingApproachAbortTilt), 0.0872665, 0.610865));
    this->landingYawRateTolerance = Clamp(NumberField(text,
        "landing_yaw_rate_tolerance_rad_s",
        this->landingYawRateTolerance), 0.0174533, 0.523599);
    this->landingContactTiltRateLimit = Clamp(NumberField(text,
        "landing_contact_tilt_rate_limit_rad_s",
        this->landingContactTiltRateLimit), 0.0174533, 0.785398);
    this->landingSettlingTiltRateLimit = Clamp(NumberField(text,
        "landing_settling_tilt_rate_limit_rad_s",
        this->landingSettlingTiltRateLimit), 0.0174533, 0.523599);
    this->landingGoAroundTiltTolerance = Clamp(NumberField(text,
        "landing_go_around_tilt_tolerance_rad",
        this->landingGoAroundTiltTolerance), 0.0174533, 0.349066);
    this->landingWaterLevel = Clamp(NumberField(text,
        "water_level_z_m", this->landingWaterLevel), -2.0, 2.0);
    this->landingPlatformTopOffset = Clamp(NumberField(text,
        "landing_platform_top_offset_m", this->landingPlatformTopOffset),
        0.05, 2.0);
    if (this->landingSurfaceMode == "water")
    {
      this->landingSurfaceZ = this->landingWaterLevel;
    }
    else
      this->landingSurfaceZ =
          this->landingWaterLevel + this->landingPlatformTopOffset;
    this->targetZ = Clamp(NumberField(text, "target_z_m", this->targetZ), 0.0, 5.0);
    this->targetRoll = NumberField(text, "target_roll_rad", this->targetRoll);
    this->targetPitch = NumberField(text, "target_pitch_rad", this->targetPitch);
    this->targetYaw = NumberField(text, "target_yaw_rad", this->targetYaw);
    const bool velocityControlEnabled = BoolField(text,
        "velocity_control_enabled", this->velocityControlEnabled);
    if (velocityControlEnabled != this->velocityControlEnabled)
    {
      this->velocityX.integral = 0.0;
      this->velocityY.integral = 0.0;
    }
    this->velocityControlEnabled = velocityControlEnabled;
    this->targetVx = NumberField(text, "target_vx_m_s", this->targetVx);
    this->targetVy = NumberField(text, "target_vy_m_s", this->targetVy);
    this->velocityTiltLimit = Clamp(NumberField(text,
        "velocity_tilt_limit_rad", this->velocityTiltLimit), 0.0, 0.785398);
    this->velocityAccelLimit = std::abs(NumberField(text,
        "velocity_accel_limit_m_s2", this->velocityAccelLimit));
    this->velocityX.kp = NumberField(text,
        "velocity_x_kp", this->velocityX.kp);
    this->velocityX.ki = NumberField(text,
        "velocity_x_ki", this->velocityX.ki);
    this->velocityX.limit = std::abs(NumberField(text,
        "velocity_x_limit", this->velocityX.limit));
    this->velocityX.integralLimit = std::abs(NumberField(text,
        "velocity_x_integral_limit", this->velocityX.integralLimit));
    this->velocityY.kp = NumberField(text,
        "velocity_y_kp", this->velocityY.kp);
    this->velocityY.ki = NumberField(text,
        "velocity_y_ki", this->velocityY.ki);
    this->velocityY.limit = std::abs(NumberField(text,
        "velocity_y_limit", this->velocityY.limit));
    this->velocityY.integralLimit = std::abs(NumberField(text,
        "velocity_y_integral_limit", this->velocityY.integralLimit));
    const bool positionControlEnabled = BoolField(text,
        "position_control_enabled", this->positionControlEnabled);
    if (positionControlEnabled != this->positionControlEnabled)
    {
      this->positionX.integral = 0.0;
      this->positionY.integral = 0.0;
    }
    this->positionControlEnabled = positionControlEnabled;
    this->targetX = NumberField(text, "target_x_m", this->targetX);
    this->targetY = NumberField(text, "target_y_m", this->targetY);
    this->positionVelocityLimit = std::abs(NumberField(text,
        "position_velocity_limit_m_s", this->positionVelocityLimit));
    this->positionX.kp = NumberField(text,
        "position_x_kp", this->positionX.kp);
    this->positionX.ki = NumberField(text,
        "position_x_ki", this->positionX.ki);
    this->positionX.kd = NumberField(text,
        "position_x_kd", this->positionX.kd);
    this->positionX.limit = std::abs(NumberField(text,
        "position_x_limit", this->positionX.limit));
    this->positionX.integralLimit = std::abs(NumberField(text,
        "position_x_integral_limit", this->positionX.integralLimit));
    this->positionY.kp = NumberField(text,
        "position_y_kp", this->positionY.kp);
    this->positionY.ki = NumberField(text,
        "position_y_ki", this->positionY.ki);
    this->positionY.kd = NumberField(text,
        "position_y_kd", this->positionY.kd);
    this->positionY.limit = std::abs(NumberField(text,
        "position_y_limit", this->positionY.limit));
    this->positionY.integralLimit = std::abs(NumberField(text,
        "position_y_integral_limit", this->positionY.integralLimit));
    this->attitudeSetpointRateLimit = Clamp(NumberField(text,
        "attitude_setpoint_rate_limit_rad_s",
        this->attitudeSetpointRateLimit), 0.0, 20.0);
    this->maxOmega = Clamp(NumberField(text, "max_omega_rad_s", this->maxOmega),
        0.0, 300.0);
    this->rotorInterferenceEnabled = BoolField(text,
        "rotor_interference_enabled", this->rotorInterferenceEnabled);
    this->coaxialMaxThrustLoss = Clamp(NumberField(text,
        "coaxial_max_thrust_loss", this->coaxialMaxThrustLoss), 0.0, 0.35);
    this->height.kp = NumberField(text, "height_kp", this->height.kp);
    this->height.ki = NumberField(text, "height_ki", this->height.ki);
    this->height.kd = NumberField(text, "height_kd", this->height.kd);
    this->height.limit = std::abs(NumberField(text, "height_limit",
        this->height.limit));
    this->roll.kp = NumberField(text, "roll_kp", this->roll.kp);
    this->roll.ki = NumberField(text, "roll_ki", this->roll.ki);
    this->roll.kd = NumberField(text, "roll_kd", this->roll.kd);
    this->roll.limit = std::abs(NumberField(text, "roll_limit", this->roll.limit));
    this->pitch.kp = NumberField(text, "pitch_kp", this->pitch.kp);
    this->pitch.ki = NumberField(text, "pitch_ki", this->pitch.ki);
    this->pitch.kd = NumberField(text, "pitch_kd", this->pitch.kd);
    this->pitch.limit = std::abs(NumberField(text, "pitch_limit", this->pitch.limit));
    this->yaw.kp = NumberField(text, "yaw_kp", this->yaw.kp);
    this->yaw.ki = NumberField(text, "yaw_ki", this->yaw.ki);
    this->yaw.kd = NumberField(text, "yaw_kd", this->yaw.kd);
    this->yaw.limit = std::abs(NumberField(text, "yaw_limit", this->yaw.limit));
    this->yawLargeKp =
        NumberField(text, "yaw_large_signal_kp", this->yawLargeKp);
    this->yawLargeKd =
        NumberField(text, "yaw_large_signal_kd", this->yawLargeKd);
    this->yawScheduleStart = Clamp(NumberField(text,
        "yaw_schedule_start_rad", this->yawScheduleStart), 0.0, M_PI);
    this->yawScheduleEnd = Clamp(NumberField(text,
        "yaw_schedule_end_rad", this->yawScheduleEnd),
        this->yawScheduleStart + 1e-4, M_PI);
    const auto previousSeed = this->nonidealitiesSeed;
    const double previousAttitudeBiasStd = this->attitudeBiasStd;
    const double previousGyroBiasStd = this->gyroBiasStd;
    this->nonidealitiesEnabled = BoolField(text,
        "nonidealities_enabled", this->nonidealitiesEnabled);
    this->attitudeNoiseStd = std::abs(NumberField(text,
        "attitude_noise_std_rad", this->attitudeNoiseStd));
    this->gyroNoiseStd = std::abs(NumberField(text,
        "gyro_noise_std_rad_s", this->gyroNoiseStd));
    this->attitudeBiasStd = std::abs(NumberField(text,
        "attitude_bias_std_rad", this->attitudeBiasStd));
    this->gyroBiasStd = std::abs(NumberField(text,
        "gyro_bias_std_rad_s", this->gyroBiasStd));
    this->positionNoiseStd = std::abs(NumberField(text,
        "position_noise_std_m", this->positionNoiseStd));
    this->velocityNoiseStd = std::abs(NumberField(text,
        "velocity_noise_std_m_s", this->velocityNoiseStd));
    this->controlDelay = Clamp(NumberField(text,
        "control_delay_s", this->controlDelay), 0.0, 0.5);
    this->motorTimeConstant = Clamp(NumberField(text,
        "motor_time_constant_s", this->motorTimeConstant), 0.0, 1.0);
    this->motorRateLimit = std::abs(NumberField(text,
        "motor_rate_limit_rad_s2", this->motorRateLimit));
    this->motorEffectiveness = Clamp(NumberField(text,
        "motor_effectiveness", this->motorEffectiveness), 0.5, 1.0);
    this->nonidealitiesSeed = static_cast<std::uint32_t>(std::max(0.0,
        NumberField(text, "nonidealities_seed", this->nonidealitiesSeed)));
    if (previousSeed != this->nonidealitiesSeed ||
        previousAttitudeBiasStd != this->attitudeBiasStd ||
        previousGyroBiasStd != this->gyroBiasStd)
    {
      this->ResetNoiseGenerator();
    }
  }

  private: void PublishStatus(const gz::sim::UpdateInfo &_info,
      const gz::math::Pose3d &_pose,
      const gz::math::Vector3d &_linearVel,
      const gz::math::Vector3d &_bodyRates, const ConfigSnapshot &_cfg,
      double _requestedOmega, double _appliedOmega,
      double _upperOmega, double _lowerOmega,
      const gz::math::Vector3d &_requestedTorque,
      const gz::math::Vector3d &_appliedTorque,
      bool _enabled, double _yawLargeSignalBlend = 0.0)
  {
    const double simTime = std::chrono::duration<double>(_info.simTime).count();
    if (simTime - this->lastStatusTime < 0.003)
      return;
    this->lastStatusTime = simTime;

    const auto euler = _pose.Rot().Euler();
    std::ostringstream out;
    out << "{\"enabled\":" << (_enabled ? "true" : "false")
        << ",\"z_m\":" << _pose.Z()
        << ",\"z_rate_m_s\":" << _linearVel.Z()
        << ",\"world_vx_m_s\":" << _linearVel.X()
        << ",\"world_vy_m_s\":" << _linearVel.Y()
        << ",\"world_x_m\":" << _pose.X()
        << ",\"world_y_m\":" << _pose.Y()
        << ",\"roll_rad\":" << euler.X()
        << ",\"pitch_rad\":" << euler.Y()
        << ",\"yaw_rad\":" << euler.Z()
        << ",\"roll_rate_rad_s\":" << _bodyRates.X()
        << ",\"pitch_rate_rad_s\":" << _bodyRates.Y()
        << ",\"yaw_rate_rad_s\":" << _bodyRates.Z()
        << ",\"measured_x_m\":" << _cfg.measuredPosition.X()
        << ",\"measured_y_m\":" << _cfg.measuredPosition.Y()
        << ",\"measured_z_m\":" << _cfg.measuredPosition.Z()
        << ",\"measured_vx_m_s\":" << _cfg.measuredLinearVelocity.X()
        << ",\"measured_vy_m_s\":" << _cfg.measuredLinearVelocity.Y()
        << ",\"measured_vz_m_s\":" << _cfg.measuredLinearVelocity.Z()
        << ",\"measured_roll_rad\":" << _cfg.measuredEuler.X()
        << ",\"measured_pitch_rad\":" << _cfg.measuredEuler.Y()
        << ",\"measured_yaw_rad\":" << _cfg.measuredEuler.Z()
        << ",\"measured_roll_rate_rad_s\":" << _cfg.measuredBodyRates.X()
        << ",\"measured_pitch_rate_rad_s\":" << _cfg.measuredBodyRates.Y()
        << ",\"measured_yaw_rate_rad_s\":" << _cfg.measuredBodyRates.Z()
        << ",\"target_roll_rad\":" << _cfg.commandRoll
        << ",\"target_pitch_rad\":" << _cfg.commandPitch
        << ",\"target_yaw_rad\":" << _cfg.commandYaw
        << ",\"filtered_target_roll_rad\":" << _cfg.targetRoll
        << ",\"filtered_target_pitch_rad\":" << _cfg.targetPitch
        << ",\"filtered_target_yaw_rad\":" << _cfg.targetYaw
        << ",\"velocity_control_enabled\":"
        << (_cfg.velocityControlEnabled ? "true" : "false")
        << ",\"target_vx_m_s\":" << _cfg.targetVx
        << ",\"target_vy_m_s\":" << _cfg.targetVy
        << ",\"velocity_accel_x_cmd_m_s2\":" << _cfg.velocityAccelX
        << ",\"velocity_accel_y_cmd_m_s2\":" << _cfg.velocityAccelY
        << ",\"velocity_accel_limit_m_s2\":" << _cfg.velocityAccelLimit
        << ",\"position_control_enabled\":"
        << (_cfg.positionControlEnabled ? "true" : "false")
        << ",\"target_x_m\":" << _cfg.targetX
        << ",\"target_y_m\":" << _cfg.targetY
        << ",\"position_velocity_x_cmd_m_s\":" << _cfg.targetVx
        << ",\"position_velocity_y_cmd_m_s\":" << _cfg.targetVy
        << ",\"position_velocity_limit_m_s\":" << _cfg.positionVelocityLimit
        << ",\"requested_motor_omega_rad_s\":" << _requestedOmega
        << ",\"motor_omega_rad_s\":" << _appliedOmega
        << ",\"upper_motor_rad_s\":" << _upperOmega
        << ",\"lower_motor_rad_s\":" << _lowerOmega
        << ",\"requested_roll_torque_nm\":" << _requestedTorque.X()
        << ",\"requested_pitch_torque_nm\":" << _requestedTorque.Y()
        << ",\"requested_yaw_torque_nm\":" << _requestedTorque.Z()
        << ",\"roll_torque_nm\":" << _appliedTorque.X()
        << ",\"pitch_torque_nm\":" << _appliedTorque.Y()
        << ",\"yaw_torque_nm\":" << _appliedTorque.Z()
        << ",\"nonidealities_enabled\":"
        << (_cfg.nonidealitiesEnabled ? "true" : "false")
        << ",\"control_delay_s\":" << _cfg.controlDelay
        << ",\"motor_time_constant_s\":" << _cfg.motorTimeConstant
        << ",\"motor_rate_limit_rad_s2\":" << _cfg.motorRateLimit
        << ",\"motor_effectiveness\":" << _cfg.motorEffectiveness
        << ",\"rotor_interference_compensation_enabled\":"
        << (_cfg.rotorInterferenceEnabled ? "true" : "false")
        << ",\"coaxial_max_thrust_loss\":" << _cfg.coaxialMaxThrustLoss
        << ",\"buoyancy_compensation_n\":" << _cfg.filteredBuoyancy
        << ",\"water_contact\":"
        << (_cfg.waterContact ? "true" : "false")
        << ",\"left_float_submerged_fraction\":" << _cfg.leftSubmerged
        << ",\"right_float_submerged_fraction\":" << _cfg.rightSubmerged
        << ",\"slamming_force_n\":" << _cfg.slammingForce
        << ",\"landing_active\":"
        << (_cfg.landingActive ? "true" : "false")
        << ",\"landing_state\":\"" << _cfg.landingState << "\""
        << ",\"landing_state_time_s\":"
        << std::max(0.0, simTime - _cfg.landingStateStart)
        << ",\"landing_target_z_m\":" << _cfg.targetZ
        << ",\"landing_target_x_m\":" << _cfg.landingTargetX
        << ",\"landing_target_y_m\":" << _cfg.landingTargetY
        << ",\"landing_target_yaw_rad\":" << _cfg.landingTargetYaw
        << ",\"landing_approach_z_m\":" << _cfg.landingMissionHoverZ
        << ",\"landing_started_on_water\":"
        << (_cfg.landingStartedOnWater ? "true" : "false")
        << ",\"landing_started_on_surface\":"
        << (_cfg.landingStartedOnWater ? "true" : "false")
        << ",\"landing_surface_mode\":\""
        << _cfg.landingSurfaceMode << "\""
        << ",\"landing_surface_z_m\":" << _cfg.landingSurfaceZ
        << ",\"landing_platform_top_offset_m\":"
        << _cfg.landingPlatformTopOffset
        << ",\"landing_geometry_description_version\":\"unified_v1\""
        << ",\"landing_vehicle_geometry_ready\":"
        << (_cfg.landingVehicleGeometryReady ? "true" : "false")
        << ",\"landing_surface_geometry_ready\":"
        << (_cfg.landingSurfaceGeometryReady ? "true" : "false")
        << ",\"landing_float_bottom_offset_m\":"
        << _cfg.landingFloatBottomOffset
        << ",\"landing_water_equilibrium_body_offset_m\":"
        << _cfg.landingWaterEquilibriumBodyOffset
        << ",\"landing_platform_safe_half_length_m\":"
        << _cfg.landingPlatformSafeHalfLength
        << ",\"landing_platform_safe_half_width_m\":"
        << _cfg.landingPlatformSafeHalfWidth
        << ",\"landing_contact_min_clearance_m\":"
        << _cfg.landingContactMinClearance
        << ",\"landing_contact_max_clearance_m\":"
        << _cfg.landingContactMaxClearance
        << ",\"landing_platform_available\":"
        << (_cfg.landingPlatformAvailable ? "true" : "false")
        << ",\"landing_platform_contact\":"
        << (_cfg.landingPlatformContact ? "true" : "false")
        << ",\"landing_speed_profile\":\"adaptive_distance_v1\""
        << ",\"landing_approach_speed_m_s\":"
        << _cfg.landingApproachSpeed
        << ",\"landing_cruise_speed_m_s\":" << _cfg.landingCruiseSpeed
        << ",\"landing_advanced_config_version\":\"configurable_v1\""
        << ",\"landing_departure_horizontal_speed_limit_m_s\":"
        << _cfg.landingDepartureHorizontalSpeedLimit
        << ",\"landing_near_horizontal_speed_limit_m_s\":"
        << _cfg.landingNearHorizontalSpeedLimit
        << ",\"landing_moving_target_correction_reserve_m_s\":"
        << _cfg.landingMovingTargetCorrectionReserve
        << ",\"landing_approach_braking_accel_m_s2\":"
        << _cfg.landingApproachBrakingAccel
        << ",\"landing_departure_tilt_limit_rad\":"
        << _cfg.landingDepartureTiltLimit
        << ",\"landing_approach_tilt_limit_rad\":"
        << _cfg.landingApproachTiltLimit
        << ",\"landing_near_tilt_limit_rad\":"
        << _cfg.landingNearTiltLimit
        << ",\"landing_warning_tilt_rad\":" << _cfg.landingWarningTilt
        << ",\"landing_abort_tilt_rad\":" << _cfg.landingAbortTilt
        << ",\"landing_approach_abort_tilt_rad\":"
        << _cfg.landingApproachAbortTilt
        << ",\"landing_abort_position_error_m\":"
        << _cfg.landingAbortPositionError
        << ",\"landing_near_max_descent_speed_m_s\":"
        << _cfg.landingNearMaxDescentSpeed
        << ",\"landing_go_around_height_m\":" << _cfg.landingGoAroundHeight
        << ",\"landing_contact_loss_grace_s\":"
        << _cfg.landingContactLossGrace
        << ",\"landing_contact_submerged_fraction\":"
        << _cfg.landingContactSubmergedFraction
        << ",\"landing_moving_target_enabled\":"
        << (_cfg.landingMovingTargetEnabled ? "true" : "false")
        << ",\"landing_target_vx_m_s\":" << _cfg.landingTargetVx
        << ",\"landing_target_vy_m_s\":" << _cfg.landingTargetVy
        << ",\"landing_target_yaw_rate_rad_s\":"
        << _cfg.landingTargetYawRate
        << ",\"landing_target_healthy\":"
        << (_cfg.landingTargetHealthy ? "true" : "false")
        << ",\"landing_target_status_age_s\":"
        << _cfg.landingTargetStatusAge
        << ",\"landing_horizontal_error_m\":"
        << std::hypot(
             _cfg.landingTargetX - _cfg.measuredPosition.X(),
             _cfg.landingTargetY - _cfg.measuredPosition.Y())
        << ",\"landing_yaw_error_rad\":"
        << std::abs(WrapPi(
             _cfg.landingTargetYaw - _cfg.measuredEuler.Z()))
        << ",\"landing_target_vz_m_s\":" << _cfg.targetVz
        << ",\"float_clearance_m\":" << _cfg.floatClearance
        << ",\"float_signed_clearance_m\":"
        << _cfg.floatSignedClearance
        << ",\"landing_peak_impact_n\":" << _cfg.landingPeakImpact
        << ",\"landing_impact_impulse_n_s\":" << _cfg.landingImpactImpulse
        << ",\"landing_touchdown_vz_m_s\":" << _cfg.touchdownVz
        << ",\"landing_touchdown_horizontal_error_m\":"
        << _cfg.touchdownHorizontalError
        << ",\"landing_touchdown_relative_speed_m_s\":"
        << _cfg.touchdownRelativeSpeed
        << ",\"landing_touchdown_yaw_error_rad\":"
        << _cfg.touchdownYawError
        << ",\"landing_dual_contact_delay_s\":"
        << _cfg.dualContactDelay
        << ",\"landing_abort_reason\":\"" << _cfg.abortReason << "\""
        << ",\"landing_abort_trigger_state\":\""
        << _cfg.abortTriggerState << "\""
        << ",\"landing_abort_measured_value\":"
        << _cfg.abortMeasuredValue
        << ",\"landing_abort_limit_value\":" << _cfg.abortLimitValue
        << ",\"yaw_large_signal_blend\":" << _yawLargeSignalBlend
        << ",\"effective_yaw_kp\":" << _cfg.yaw.kp
        << ",\"effective_yaw_kd\":" << _cfg.yaw.kd
        << ",\"sim_time_s\":" << simTime << "}";
    gz::msgs::StringMsg msg;
    msg.set_data(out.str());
    this->statusPub.Publish(msg);
    this->landingStatusPub.Publish(msg);
  }

  private: gz::sim::Model model;
  private: gz::sim::Link link;
  private: gz::sim::Joint upperJoint;
  private: gz::sim::Joint lowerJoint;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher statusPub;
  private: gz::transport::Node::Publisher landingStatusPub;
  private: gz::transport::Node::Publisher motorCommandPub;
  private: std::mutex mutex;

  private: double mass{8.2};
  private: double gravity{9.8};
  private: double thrustCoeff{2.1610671e-3};
  private: bool enabled{false};
  private: double targetZ{0.8};
  private: double targetRoll{0.0};
  private: double targetPitch{0.0};
  private: double targetYaw{0.0};
  private: double filteredTargetRoll{0.0};
  private: double filteredTargetPitch{0.0};
  private: double filteredTargetYaw{0.0};
  private: double attitudeSetpointRateLimit{0.75};
  private: bool velocityControlEnabled{false};
  private: double targetVx{0.0};
  private: double targetVy{0.0};
  private: double velocityTiltLimit{0.261799};
  private: double velocityAccelLimit{2.2};
  private: Pid velocityX{2.8, 0.15, 0.0, 2.2, 1.0};
  private: Pid velocityY{2.8, 0.15, 0.0, 2.2, 1.0};
  private: bool positionControlEnabled{false};
  private: double targetX{0.0};
  private: double targetY{0.0};
  private: double positionVelocityLimit{2.5};
  private: Pid positionX{2.5, 0.0, 0.95, 2.5, 2.0};
  private: Pid positionY{2.5, 0.0, 0.95, 2.5, 2.0};
  private: double maxOmega{156.0};
  private: bool rotorInterferenceEnabled{true};
  private: double coaxialMaxThrustLoss{0.06};
  private: Pid height{80.0, 0.0, 45.0, 40.0, 0.5};
  private: Pid roll{193.3, 0.0, 8.61, 2.5, 0.2};
  private: Pid pitch{351.3, 0.0, 15.65, 2.7, 0.2};
  private: Pid yaw{296.1, 0.0, 13.19, 0.7, 0.2};
  private: double yawLargeKp{20.0};
  private: double yawLargeKd{3.0};
  private: double yawScheduleStart{0.02};
  private: double yawScheduleEnd{0.08};
  private: bool yawLargeSignalMode{false};
  private: bool nonidealitiesEnabled{false};
  private: double attitudeNoiseStd{0.000349066};
  private: double gyroNoiseStd{0.00174533};
  private: double attitudeBiasStd{0.000872665};
  private: double gyroBiasStd{0.000872665};
  private: double positionNoiseStd{0.003};
  private: double velocityNoiseStd{0.01};
  private: double controlDelay{0.015};
  private: double motorTimeConstant{0.08};
  private: double motorRateLimit{500.0};
  private: double motorEffectiveness{0.98};
  private: std::uint32_t nonidealitiesSeed{20260726u};
  private: std::mt19937 rng{20260726u};
  private: gz::math::Vector3d attitudeBias{0.0, 0.0, 0.0};
  private: gz::math::Vector3d gyroBias{0.0, 0.0, 0.0};
  private: std::deque<ActuatorCommand> commandQueue;
  private: ActuatorCommand lastDelayedCommand;
  private: double appliedMotorOmega{0.0};
  private: bool waterContact{false};
  private: double waterBuoyancy{0.0};
  private: double filteredBuoyancy{0.0};
  private: double waterSlammingForce{0.0};
  private: double leftSubmerged{0.0};
  private: double rightSubmerged{0.0};
  private: bool landingStartRequested{false};
  private: bool landingLocalLandRequested{false};
  private: bool landingLocalLandMode{false};
  private: bool landingActive{false};
  private: std::string landingState{"IDLE"};
  private: std::string landingAbortReason;
  private: std::string landingAbortTriggerState;
  private: double landingAbortMeasuredValue{0.0};
  private: double landingAbortLimitValue{0.0};
  private: double landingStateStart{0.0};
  private: double landingStableSince{-1.0};
  private: double landingHighHoverZ{1.8};
  private: double landingConfiguredTargetX{0.0};
  private: double landingConfiguredTargetY{0.0};
  private: double landingConfiguredTargetYaw{0.0};
  private: double landingTargetX{0.0};
  private: double landingTargetY{0.0};
  private: double landingTargetYaw{0.0};
  private: bool landingMovingTargetEnabled{false};
  private: double landingTargetVx{0.0};
  private: double landingTargetVy{0.0};
  private: double landingTargetYawRate{0.0};
  private: double landingTargetStatusTimeout{0.30};
  private: double landingTargetSpeedLimit{0.80};
  private: bool landingTargetStatusValid{false};
  private: double landingTargetStatusSimTime{-1.0};
  private: double landingTargetStatusAge{0.0};
  private: bool landingTargetHealthy{true};
  private: bool landingTargetTrackingFrozen{false};
  private: double landingMissionStartTime{0.0};
  private: double landingDepartureX{0.0};
  private: double landingDepartureY{0.0};
  private: double landingDepartureYaw{0.0};
  private: double landingMissionHoverZ{1.8};
  private: bool landingStartedOnWater{false};
  private: double landingApproachSpeed{0.8};
  private: double landingCruiseSpeed{2.5};
  private: double landingPositionTolerance{0.15};
  private: double landingYawTolerance{0.0872665};
  private: double landingDescentRate{0.35};
  private: double landingFlareClearance{0.40};
  private: double landingFlareRate{0.12};
  private: double landingTouchdownMaxVz{0.20};
  private: double landingContactConfirmTime{0.30};
  private: double landingSpoolDownTime{1.50};
  private: double landingWaterLevel{0.0};
  private: double landingPlatformTopOffset{0.20};
  private: std::string landingSurfaceMode{"water"};
  private: double landingSurfaceZ{0.0};
  private: bool landingVehicleGeometryReady{false};
  private: bool landingSurfaceGeometryReady{false};
  private: double landingFloatBottomOffset{0.0};
  private: double landingFloatFootprintHalfLength{0.0};
  private: double landingFloatFootprintHalfWidth{0.0};
  private: double landingWaterEquilibriumBodyOffset{0.0};
  private: double landingPlatformHalfLength{0.0};
  private: double landingPlatformHalfWidth{0.0};
  private: double landingPlatformEdgeMargin{0.0};
  private: double landingContactMinClearance{0.0};
  private: double landingContactMaxClearance{0.0};
  private: bool landingPlatformAvailable{false};
  private: double landingDepartureClearanceMargin{0.10};
  private: double landingDepartureHorizontalSpeedLimit{0.30};
  private: double landingNearHorizontalSpeedLimit{0.30};
  private: double landingMovingTargetCorrectionReserve{0.30};
  private: double landingApproachBrakingAccel{0.55};
  private: double landingWarningTilt{0.0872665};
  private: double landingAbortTilt{0.139626};
  private: double landingApproachAbortTilt{0.209440};
  private: double landingAbortPositionError{0.40};
  private: double landingNearMaxDescentSpeed{0.30};
  private: double landingDepartureTiltLimit{0.0872665};
  private: double landingApproachTiltLimit{0.174533};
  private: double landingNearTiltLimit{0.0872665};
  private: double landingGoAroundHeight{1.0};
  private: double landingDepartureStableTime{0.50};
  private: double landingAlignStableTime{0.80};
  private: double landingHoverStableTime{1.0};
  private: double landingApproachRelativeSpeedTolerance{0.15};
  private: double landingAlignRelativeSpeedTolerance{0.10};
  private: double landingHoverRelativeSpeedTolerance{0.12};
  private: double landingDepartureHorizontalSpeedTolerance{0.10};
  private: double landingHeightTolerance{0.16};
  private: double landingApproachVerticalSpeedTolerance{0.10};
  private: double landingPrecisionVerticalSpeedTolerance{0.08};
  private: double landingNearOverspeedGrace{1.0};
  private: double landingContactSubmergedFraction{0.02};
  private: double landingYawRateTolerance{0.0872665};
  private: double landingContactTiltRateLimit{0.174533};
  private: double landingSettlingTiltRateLimit{0.0872665};
  private: double landingSettlingVerticalSpeedLimit{0.08};
  private: double landingSettlingTime{0.50};
  private: double landingContactLossGrace{0.08};
  private: double landingContactUnstableSince{-1.0};
  private: double landingGoAroundHeightTolerance{0.18};
  private: double landingGoAroundVerticalSpeedTolerance{0.12};
  private: double landingGoAroundTiltTolerance{0.0872665};
  private: double landingFlareTransitionMargin{0.02};
  private: double landingTargetVz{0.0};
  private: double landingSpoolStartOmega{0.0};
  private: double landingSpoolOmega{0.0};
  private: double landingPeakImpact{0.0};
  private: double landingImpactImpulse{0.0};
  private: double landingTouchdownVz{0.0};
  private: double landingTouchdownHorizontalError{0.0};
  private: double landingTouchdownRelativeSpeed{0.0};
  private: double landingTouchdownYawError{0.0};
  private: double landingFirstContactTime{-1.0};
  private: double landingDualContactDelay{0.0};
  private: bool landingLimitsSaved{false};
  private: double landingSavedPositionVelocityLimit{2.5};
  private: double landingSavedVelocityTiltLimit{0.261799};
  private: double lastStatusTime{-1.0};
};
}

GZ_ADD_PLUGIN(coaxial_uav::CoaxialPidController,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(coaxial_uav::CoaxialPidController,
    "coaxial_uav::CoaxialPidController")
