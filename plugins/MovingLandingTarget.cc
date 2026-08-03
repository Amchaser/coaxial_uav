#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
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

double WrapPi(double _angle)
{
  constexpr double kPi = 3.14159265358979323846;
  return std::remainder(_angle, 2.0 * kPi);
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
}

class MovingLandingTarget:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    if (_sdf && _sdf->HasElement("role"))
      this->role = _sdf->Get<std::string>("role");
    if (_sdf && _sdf->HasElement("publish_status"))
      this->publishStatus = _sdf->Get<bool>("publish_status");
    if (_sdf && _sdf->HasElement("platform_available"))
      this->platformAvailable = _sdf->Get<bool>("platform_available");
    if (_sdf && _sdf->HasElement("platform_top_offset_m"))
      this->platformTopOffset = _sdf->Get<double>("platform_top_offset_m");
    if (_sdf && _sdf->HasElement("platform_hidden_z_m"))
      this->platformHiddenZ = _sdf->Get<double>("platform_hidden_z_m");
    if (_sdf && _sdf->HasElement("platform_half_length_m"))
      this->platformHalfLength = _sdf->Get<double>("platform_half_length_m");
    if (_sdf && _sdf->HasElement("platform_half_width_m"))
      this->platformHalfWidth = _sdf->Get<double>("platform_half_width_m");
    if (_sdf && _sdf->HasElement("platform_thickness_m"))
      this->platformThickness = _sdf->Get<double>("platform_thickness_m");
    if (_sdf && _sdf->HasElement("platform_edge_margin_m"))
      this->platformEdgeMargin = _sdf->Get<double>("platform_edge_margin_m");
    if (_sdf && _sdf->HasElement("contact_min_clearance_m"))
      this->contactMinClearance = _sdf->Get<double>("contact_min_clearance_m");
    if (_sdf && _sdf->HasElement("contact_max_clearance_m"))
      this->contactMaxClearance = _sdf->Get<double>("contact_max_clearance_m");
    if (_sdf && _sdf->HasElement("initial_overlap_min_clearance_m"))
      this->initialOverlapMinClearance =
          _sdf->Get<double>("initial_overlap_min_clearance_m");
    this->link = gz::sim::Link(this->model.LinkByName(
        _ecm, this->role == "platform" ? "deck_link" : "link"));
    this->node.Subscribe("/coaxial_uav/control/config",
        &MovingLandingTarget::OnConfig, this);
    this->node.Subscribe("/coaxial_uav/control/status",
        &MovingLandingTarget::OnControlStatus, this);
    if (this->publishStatus)
      this->statusPub = this->node.Advertise<gz::msgs::StringMsg>(
          "/coaxial_uav/landing/target/status");
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    const double simTime =
        std::chrono::duration<double>(_info.simTime).count();
    if (dt <= 0.0 || !this->model.Valid(_ecm))
      return;

    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->resetRequested || !this->initialized)
    {
      this->x = this->configuredX;
      this->y = this->configuredY;
      this->yaw = this->configuredYaw;
      this->resetRequested = false;
      this->initialized = true;
    }
    else if (this->movingEnabled && this->landingMissionActive)
    {
      this->x += this->velocityX * dt;
      this->y += this->velocityY * dt;
      this->yaw = WrapPi(this->yaw + this->yawRate * dt);
    }

    const double surfaceZ = this->waterLevel +
        (this->surfaceMode == "platform" ? this->platformTopOffset : 0.0);
    double modelZ = this->platformHiddenZ;
    if (this->role == "platform")
    {
      if (this->surfaceMode == "platform" && this->platformDeployed)
        modelZ = surfaceZ - 0.5 * this->platformThickness;
    }
    else if (this->landingMissionActive)
    {
      modelZ = surfaceZ + 0.008;
    }
    this->model.SetWorldPoseCmd(_ecm, gz::math::Pose3d(
        this->x, this->y, modelZ,
        0.0, 0.0, this->yaw));
    if (this->role == "platform" && this->link.Valid(_ecm))
    {
      const bool movingPlatform =
          this->surfaceMode == "platform" && this->platformDeployed &&
          this->movingEnabled && this->landingMissionActive;
      const double worldVx = movingPlatform ? this->velocityX : 0.0;
      const double worldVy = movingPlatform ? this->velocityY : 0.0;
      const double c = std::cos(this->yaw);
      const double s = std::sin(this->yaw);
      this->link.SetLinearVelocity(_ecm, gz::math::Vector3d(
          c * worldVx + s * worldVy,
          -s * worldVx + c * worldVy, 0.0));
      this->link.SetAngularVelocity(_ecm, gz::math::Vector3d(
          0.0, 0.0, movingPlatform ? this->yawRate : 0.0));
    }
    if (this->publishStatus && (this->lastPublishTime < 0.0 ||
        simTime - this->lastPublishTime >= 0.01)
       )
    {
      this->PublishStatus(simTime);
      this->lastPublishTime = simTime;
    }
  }

  private: void OnConfig(const gz::msgs::StringMsg &_msg)
  {
    const auto text = _msg.data();
    std::lock_guard<std::mutex> lock(this->mutex);
    const bool landingStart = BoolField(text, "landing_start", false);
    const bool nextMissionActive = BoolField(
        text, "landing_mission_active", this->landingMissionActive);
    const bool nextMovingEnabled = BoolField(
        text, "landing_moving_target_enabled", this->movingEnabled);
    const double nextConfiguredX = Clamp(NumberField(
        text, "landing_target_x_m", this->configuredX), -100.0, 100.0);
    const double nextConfiguredY = Clamp(NumberField(
        text, "landing_target_y_m", this->configuredY), -100.0, 100.0);
    const double nextConfiguredYaw = WrapPi(NumberField(
        text, "landing_target_yaw_rad", this->configuredYaw));
    const bool targetDefinitionChanged =
        nextMovingEnabled != this->movingEnabled ||
        std::abs(nextConfiguredX - this->configuredX) > 1e-9 ||
        std::abs(nextConfiguredY - this->configuredY) > 1e-9 ||
        std::abs(WrapPi(nextConfiguredYaw - this->configuredYaw)) > 1e-9;
    this->movingEnabled = nextMovingEnabled;
    this->configuredX = nextConfiguredX;
    this->configuredY = nextConfiguredY;
    this->configuredYaw = nextConfiguredYaw;
    this->velocityX = Clamp(NumberField(
        text, "landing_target_vx_m_s", this->velocityX), -2.0, 2.0);
    this->velocityY = Clamp(NumberField(
        text, "landing_target_vy_m_s", this->velocityY), -2.0, 2.0);
    this->yawRate = Clamp(NumberField(
        text, "landing_target_yaw_rate_rad_s", this->yawRate),
        -1.0, 1.0);
    this->waterLevel = Clamp(NumberField(
        text, "water_level_z_m", this->waterLevel), -2.0, 2.0);
    this->platformTopOffset = Clamp(NumberField(
        text, "landing_platform_top_offset_m", this->platformTopOffset),
        0.05, 2.0);
    const auto requestedSurfaceMode = StringField(
        text, "landing_surface_mode", this->surfaceMode);
    this->surfaceMode = requestedSurfaceMode == "platform" ?
        "platform" : "water";
    if (landingStart || nextMissionActive)
    {
      this->landingMissionActive = true;
      this->landingControllerActiveSeen = false;
      this->platformDeployed = this->surfaceMode == "platform";
    }
    else if (!nextMissionActive)
    {
      this->landingMissionActive = false;
      this->landingControllerActiveSeen = false;
      if (this->surfaceMode != "platform")
        this->platformDeployed = false;
    }
    this->resetRequested = this->resetRequested || targetDefinitionChanged ||
        (landingStart && nextMovingEnabled);
  }

  private: void OnControlStatus(const gz::msgs::StringMsg &_msg)
  {
    const auto text = _msg.data();
    std::lock_guard<std::mutex> lock(this->mutex);
    const bool controllerLandingActive = BoolField(
        text, "landing_active", this->landingMissionActive);
    if (controllerLandingActive)
    {
      this->landingMissionActive = true;
      this->landingControllerActiveSeen = true;
      if (this->surfaceMode == "platform")
        this->platformDeployed = true;
    }
    else if (this->landingControllerActiveSeen)
    {
      this->landingMissionActive = false;
      this->landingControllerActiveSeen = false;
    }
  }

  private: void PublishStatus(double _simTime)
  {
    std::ostringstream out;
    const double surfaceZ = this->waterLevel +
        (this->surfaceMode == "platform" ? this->platformTopOffset : 0.0);
    out << "{\"valid\":true"
        << ",\"position_reset_policy\":"
        << "\"on_landing_start_or_definition_change_v2\""
        << ",\"platform_mode_version\":\"solid_deck_v1\""
        << ",\"platform_height_config_version\":\"configurable_v1\""
        << ",\"surface_geometry_version\":\"solid_deck_geometry_v1\""
        << ",\"surface_mode\":\"" << this->surfaceMode << "\""
        << ",\"surface_z_m\":" << surfaceZ
        << ",\"platform_top_z_m\":"
        << this->waterLevel + this->platformTopOffset
        << ",\"platform_top_offset_m\":" << this->platformTopOffset
        << ",\"platform_half_length_m\":" << this->platformHalfLength
        << ",\"platform_half_width_m\":" << this->platformHalfWidth
        << ",\"platform_thickness_m\":" << this->platformThickness
        << ",\"platform_edge_margin_m\":" << this->platformEdgeMargin
        << ",\"contact_min_clearance_m\":" << this->contactMinClearance
        << ",\"contact_max_clearance_m\":" << this->contactMaxClearance
        << ",\"initial_overlap_min_clearance_m\":"
        << this->initialOverlapMinClearance
        << ",\"platform_available\":"
        << (this->platformAvailable ? "true" : "false")
        << ",\"platform_deployed\":"
        << (this->platformDeployed ? "true" : "false")
        << ",\"moving_configured\":"
        << (this->movingEnabled ? "true" : "false")
        << ",\"landing_mission_active\":"
        << (this->landingMissionActive ? "true" : "false")
        << ",\"moving\":"
        << ((this->movingEnabled && this->landingMissionActive) ?
            "true" : "false")
        << ",\"x_m\":" << this->x
        << ",\"y_m\":" << this->y
        << ",\"yaw_rad\":" << this->yaw
        << ",\"vx_m_s\":"
        << ((this->movingEnabled && this->landingMissionActive) ?
            this->velocityX : 0.0)
        << ",\"vy_m_s\":"
        << ((this->movingEnabled && this->landingMissionActive) ?
            this->velocityY : 0.0)
        << ",\"yaw_rate_rad_s\":"
        << ((this->movingEnabled && this->landingMissionActive) ?
            this->yawRate : 0.0)
        << ",\"sim_time_s\":" << _simTime << "}";
    gz::msgs::StringMsg msg;
    msg.set_data(out.str());
    this->statusPub.Publish(msg);
  }

  private: gz::sim::Model model;
  private: gz::sim::Link link;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher statusPub;
  private: std::mutex mutex;
  private: bool initialized{false};
  private: bool resetRequested{true};
  private: bool movingEnabled{false};
  private: bool landingMissionActive{false};
  private: bool landingControllerActiveSeen{false};
  private: bool platformDeployed{false};
  private: bool publishStatus{true};
  private: bool platformAvailable{false};
  private: std::string role{"marker"};
  private: std::string surfaceMode{"water"};
  private: double configuredX{0.0};
  private: double configuredY{0.0};
  private: double configuredYaw{0.0};
  private: double x{0.0};
  private: double y{0.0};
  private: double yaw{0.0};
  private: double velocityX{0.0};
  private: double velocityY{0.0};
  private: double yawRate{0.0};
  private: double waterLevel{0.0};
  private: double platformTopOffset{0.20};
  private: double platformHiddenZ{-3.0};
  private: double platformHalfLength{1.20};
  private: double platformHalfWidth{0.90};
  private: double platformThickness{0.20};
  private: double platformEdgeMargin{0.03};
  private: double contactMinClearance{-0.08};
  private: double contactMaxClearance{0.05};
  private: double initialOverlapMinClearance{-0.08};
  private: double lastPublishTime{-1.0};
};
}

GZ_ADD_PLUGIN(coaxial_uav::MovingLandingTarget,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(coaxial_uav::MovingLandingTarget,
    "coaxial_uav::MovingLandingTarget")
