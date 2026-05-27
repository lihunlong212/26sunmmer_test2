#include "activity_control_pkg/route_target_publisher.hpp"

#include <angles/angles.h>

#include <chrono>
#include <clocale>
#include <cmath>
#include <functional>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

namespace
{
constexpr double kDefaultTimerPeriodSec = 0.05;
}  // namespace

RouteTargetPublisherNode::RouteTargetPublisherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("route_target_publisher", options),
  current_idx_(std::numeric_limits<std::size_t>::max()),
  has_height_(false),
  current_height_cm_(0.0),
  spray_decision_timeout_sec_(0.0),
  spray_data_stale_timeout_sec_(0.0),
  spray_required_frames_(0),
  laser_pulse_command_(0),
  mission_complete_sent_(false),
  has_spray_allowed_(false),
  latest_spray_allowed_(false),
  spray_active_(false),
  spray_allowed_frame_count_(0)
{
  pos_tol_cm_ = declare_parameter("position_tolerance_cm", 9.0);
  yaw_tol_deg_ = declare_parameter("yaw_tolerance_deg", 5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm", 12.0);
  map_frame_ = declare_parameter("map_frame", "map");
  laser_link_frame_ = declare_parameter("laser_link_frame", "laser_link");
  output_topic_ = declare_parameter("output_topic", "/target_position");
  spray_decision_timeout_sec_ = declare_parameter("spray_decision_timeout_sec", 1.5);
  spray_data_stale_timeout_sec_ = declare_parameter("spray_data_stale_timeout_sec", 0.5);
  spray_required_frames_ = declare_parameter("spray_required_frames", 1);
  laser_pulse_command_ = declare_parameter("laser_pulse_command", 3);
  declare_parameter("auto_start_route", false);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  mission_complete_pub_ =
    create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());
  laser_cmd_pub_ = create_publisher<std_msgs::msg::Int32>("/laser/cmd", rclcpp::QoS(10).reliable());

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::heightCallback, this, std::placeholders::_1));
  spray_allowed_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/spray_allowed",
    rclcpp::QoS(10),
    std::bind(&RouteTargetPublisherNode::sprayAllowedCallback, this, std::placeholders::_1));

  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(kDefaultTimerPeriodSec),
    std::bind(&RouteTargetPublisherNode::monitorTimerCallback, this));

  loadAutoStartRouteFromParameters();

  RCLCPP_INFO(
    get_logger(),
    "RouteTargetPublisher initialized: map=%s laser_link=%s topic=%s",
    map_frame_.c_str(),
    laser_link_frame_.c_str(),
    output_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Tolerances: position=%.1fcm yaw=%.1fdeg height=%.1fcm",
    pos_tol_cm_,
    yaw_tol_deg_,
    height_tol_cm_);
  RCLCPP_INFO(
    get_logger(),
    "Spray gating: frames=%d decision_timeout=%.1fs stale=%.1fs laser_cmd=%d",
    spray_required_frames_,
    spray_decision_timeout_sec_,
    spray_data_stale_timeout_sec_,
    laser_pulse_command_);
}

void RouteTargetPublisherNode::loadAutoStartRouteFromParameters()
{
  if (!get_parameter("auto_start_route").as_bool()) {
    return;
  }

  const std::vector<Target> route{
    Target{0.0, 0.0, 150.0, 0.0},
    Target{125.0, 100.0, 150.0, 0.0, true},
    Target{0.0, 0.0, 150.0, 0.0},
    Target{0.0, 0.0, 0.0, 0.0},
  };

  RCLCPP_INFO(get_logger(), "Auto-starting source-defined waypoint route with %zu targets.", route.size());
  for (std::size_t index = 0; index < route.size(); ++index) {
    const Target target = route[index];
    addTarget(target);
    RCLCPP_INFO(
      get_logger(),
      "Loaded auto waypoint %zu/%zu: x=%.1f y=%.1f z=%.1f yaw=%.1f spray=%s",
      index + 1,
      route.size(),
      target.x_cm,
      target.y_cm,
      target.z_cm,
      target.yaw_deg,
      target.spray ? "true" : "false");
  }
}

void RouteTargetPublisherNode::addTarget(const Target & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_empty = targets_.empty();
  const bool was_completed =
    current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size();
  targets_.push_back(target);
  if (was_empty || was_completed) {
    mission_complete_sent_ = false;
    resetSprayState();
    current_idx_ = was_completed ? targets_.size() - 1 : 0;
    publishCurrent();
  }
}

std::size_t RouteTargetPublisherNode::currentIndex() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_idx_;
}

std::size_t RouteTargetPublisherNode::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return targets_.size();
}

void RouteTargetPublisherNode::publishCurrent()
{
  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ < targets_.size()) {
    publishTarget(getPublishedTarget(targets_[current_idx_]), current_idx_ == 0);
  }
}

void RouteTargetPublisherNode::publishTarget(const Target & target, bool init_flag)
{
  std_msgs::msg::Float32MultiArray message;
  message.data.resize(4);
  message.data[0] = static_cast<float>(target.x_cm);
  message.data[1] = static_cast<float>(target.y_cm);
  message.data[2] = static_cast<float>(target.z_cm);
  message.data[3] = static_cast<float>(target.yaw_deg);
  target_pub_->publish(message);

  std_msgs::msg::UInt8 active_msg;
  active_msg.data = 2;
  active_controller_pub_->publish(active_msg);

  RCLCPP_INFO(
    get_logger(),
    "Published target: x=%.1fcm y=%.1fcm z=%.1fcm yaw=%.1fdeg spray=%s%s",
    target.x_cm,
    target.y_cm,
    target.z_cm,
    target.yaw_deg,
    target.spray ? "true" : "false",
    init_flag ? " (first)" : "");
}

Target RouteTargetPublisherNode::getPublishedTarget(const Target & target) const
{
  return target;
}

void RouteTargetPublisherNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

void RouteTargetPublisherNode::sprayAllowedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  latest_spray_allowed_ = msg->data;
  has_spray_allowed_ = true;
  last_spray_allowed_time_ = now();
}

bool RouteTargetPublisherNode::getCurrentPose(
  double & x_cm,
  double & y_cm,
  double & z_cm,
  double & yaw_deg)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    x_cm = meterToCm(transform.transform.translation.x);
    y_cm = meterToCm(transform.transform.translation.y);
    z_cm = has_height_ ? current_height_cm_ : 0.0;

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "TF lookup failed (%s -> %s): %s",
      map_frame_.c_str(),
      laser_link_frame_.c_str(),
      ex.what());
    return false;
  }
}

bool RouteTargetPublisherNode::isReached(
  const Target & target,
  double x_cm,
  double y_cm,
  double z_cm,
  double yaw_deg) const
{
  const double dx = target.x_cm - x_cm;
  const double dy = target.y_cm - y_cm;
  const double dxy = std::hypot(dx, dy);
  const double dz = target.z_cm - z_cm;
  const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);

  const bool z_ok = std::fabs(dz) <= height_tol_cm_;
  const bool xy_ok = dxy <= pos_tol_cm_;
  const bool yaw_ok = std::fabs(dyaw) <= yaw_tol_deg_;

  if (target.z_cm > 20.0) {
    if (current_idx_ == 0) {
      return z_ok;
    }
    return z_ok && xy_ok;
  }

  return z_ok && xy_ok && yaw_ok;
}

void RouteTargetPublisherNode::advanceToNextTarget()
{
  resetSprayState();
  ++current_idx_;
  if (current_idx_ < targets_.size()) {
    publishCurrent();
  } else {
    current_idx_ = targets_.size();
    if (!mission_complete_sent_ && mission_complete_pub_) {
      std_msgs::msg::Empty mission_complete_msg;
      mission_complete_pub_->publish(mission_complete_msg);
      mission_complete_sent_ = true;
    }
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    RCLCPP_INFO(get_logger(), "All targets completed.");
  }
}

void RouteTargetPublisherNode::resetSprayState()
{
  spray_active_ = false;
  spray_allowed_frame_count_ = 0;
  spray_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

bool RouteTargetPublisherNode::handleSprayTarget(const rclcpp::Time & now_time)
{
  if (!spray_active_) {
    spray_active_ = true;
    spray_allowed_frame_count_ = 0;
    spray_start_time_ = now_time;
    RCLCPP_INFO(
      get_logger(),
      "Spray decision started for target %zu.",
      current_idx_);
  }

  const double elapsed = (now_time - spray_start_time_).seconds();
  if (
    !has_spray_allowed_ ||
    last_spray_allowed_time_.nanoseconds() == 0 ||
    (now_time - last_spray_allowed_time_).seconds() > spray_data_stale_timeout_sec_)
  {
    spray_allowed_frame_count_ = 0;
    if (elapsed >= spray_decision_timeout_sec_) {
      RCLCPP_WARN(
        get_logger(),
        "No fresh /spray_allowed for target %zu after %.1fs. Skipping spray.",
        current_idx_,
        elapsed);
      advanceToNextTarget();
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Waiting for fresh /spray_allowed for target %zu.",
      current_idx_);
    return true;
  }

  if (latest_spray_allowed_) {
    ++spray_allowed_frame_count_;
    if (spray_allowed_frame_count_ >= spray_required_frames_) {
      std_msgs::msg::Int32 laser_msg;
      laser_msg.data = laser_pulse_command_;
      laser_cmd_pub_->publish(laser_msg);
      RCLCPP_INFO(
        get_logger(),
        "Spraying target %zu with /laser/cmd=%d.",
        current_idx_,
        laser_pulse_command_);
      advanceToNextTarget();
    }
    return true;
  }

  spray_allowed_frame_count_ = 0;
  RCLCPP_INFO(
    get_logger(),
    "Target %zu is not green according to /spray_allowed. Skipping spray.",
    current_idx_);
  advanceToNextTarget();
  return true;
}

void RouteTargetPublisherNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (current_idx_ != std::numeric_limits<std::size_t>::max() && current_idx_ >= targets_.size()) {
    std_msgs::msg::UInt8 active_msg;
    active_msg.data = 3;
    active_controller_pub_->publish(active_msg);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "All targets completed. Keeping stop signal active.");
    return;
  }

  if (current_idx_ == std::numeric_limits<std::size_t>::max()) {
    return;
  }

  double x_cm = 0.0;
  double y_cm = 0.0;
  double z_cm = 0.0;
  double yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, z_cm, yaw_deg)) {
    return;
  }

  const Target & target = targets_[current_idx_];
  const rclcpp::Time now_time = now();

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    5000,
    "Current target %zu: x=%.1f y=%.1f z=%.1f yaw=%.1f spray=%s",
    current_idx_,
    target.x_cm,
    target.y_cm,
    target.z_cm,
    target.yaw_deg,
    target.spray ? "true" : "false");

  if (isReached(target, x_cm, y_cm, z_cm, yaw_deg)) {
    const double dx = target.x_cm - x_cm;
    const double dy = target.y_cm - y_cm;
    const double dz = target.z_cm - z_cm;
    const double dyaw = normalizeAngleDeg(target.yaw_deg - yaw_deg);
    RCLCPP_INFO(
      get_logger(),
      "Target %zu reached: pos_err=(%.1f, %.1f, %.1f)cm yaw_err=%.1fdeg current=(%.1f, %.1f, %.1f, %.1f)",
      current_idx_,
      dx,
      dy,
      dz,
      dyaw,
      x_cm,
      y_cm,
      z_cm,
      yaw_deg);
    if (target.spray && handleSprayTarget(now_time)) {
      return;
    }
    advanceToNextTarget();
  }
}

double RouteTargetPublisherNode::meterToCm(double value_m)
{
  return value_m * 100.0;
}

double RouteTargetPublisherNode::radToDeg(double value_rad)
{
  return value_rad * 180.0 / M_PI;
}

double RouteTargetPublisherNode::normalizeAngleDeg(double angle_deg) const
{
  const double normalized = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(normalized);
}

RouteTestNode::RouteTestNode(
  const std::shared_ptr<RouteTargetPublisherNode> & route_node,
  const rclcpp::NodeOptions & options)
: rclcpp::Node("route_test_node", options),
  route_node_(route_node),
  route_locked_(false)
{
  std::setlocale(LC_ALL, "");

  routes_ = buildRoutes();
  route_choice_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/route_choice",
    rclcpp::QoS(10),
    std::bind(&RouteTestNode::routeChoiceCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Route selection node is waiting on /route_choice. Available routes: %zu",
    routes_.size());
}

void RouteTestNode::routeChoiceCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  const RouteId route_id = msg->data;
  if (route_locked_) {
    RCLCPP_INFO(
      get_logger(),
      "Ignoring /route_choice=%u because a route is already active or has already started.",
      static_cast<unsigned>(route_id));
    return;
  }

  const auto route_it = routes_.find(route_id);
  if (route_it == routes_.end()) {
    RCLCPP_WARN(
      get_logger(),
      "Received unsupported /route_choice=%u. Route will not start.",
      static_cast<unsigned>(route_id));
    return;
  }

  if (route_it->second.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Received /route_choice=%u, but the route is empty. Ignoring.",
      static_cast<unsigned>(route_id));
    return;
  }

  loadRoute(route_id, route_it->second);
}

std::unordered_map<RouteTestNode::RouteId, std::vector<Target>> RouteTestNode::buildRoutes() const
{
  std::unordered_map<RouteId, std::vector<Target>> routes;

  routes.emplace(RouteId{1}, std::vector<Target>{
    Target{0.0, 0.0, 145.0, 0.0},

    Target{200.0, -50.0, 145.0, 0.0, true},
    Target{250.0, -50.0, 145.0, 0.0, true},

    Target{250.0, -100.0, 145.0, 0.0, true},
    Target{200.0, -100.0, 145.0, 0.0, true},

    Target{200.0, -150.0, 145.0, 0.0, true},
    Target{250.0, -150.0, 145.0, 0.0, true},

    Target{250.0, -200.0, 145.0, 0.0, true},
    Target{250.0, -250.0, 145.0, 0.0, true},
    Target{250.0, -300.0, 145.0, 0.0, true},
    Target{250.0, -350.0, 145.0, 0.0, true},
 
    Target{200.0, -350.0, 145.0, 0.0, true},
    Target{200.0, -300.0, 145.0, 0.0, true},
    Target{200.0, -250.0, 145.0, 0.0, true},
    Target{200.0, -200.0, 145.0, 0.0, true},

    Target{150.0, -200.0, 145.0, 0.0, true},
    Target{150.0, -250.0, 145.0, 0.0, true},
    Target{150.0, -300.0, 145.0, 0.0, true},
    Target{150.0, -350.0, 145.0, 0.0, true},

    Target{100.0, -350.0, 145.0, 0.0, true},
    Target{100.0, -300.0, 145.0, 0.0, true},
    Target{100.0, -250.0, 145.0, 0.0, true},
    Target{100.0, -200.0, 145.0, 0.0, true},

    Target{50.0, -200.0, 145.0, 0.0, true},
    Target{50.0, -250.0, 145.0, 0.0, true},
    Target{50.0, -300.0, 145.0, 0.0, true},
    Target{50.0, -350.0, 145.0, 0.0, true},

    Target{0.0, -350.0, 145.0, 0.0, true},
    Target{0.0, -300.0, 145.0, 0.0, true},
    Target{0.0, -250.0, 145.0, 0.0, true},
    Target{0.0, -200.0, 145.0, 0.0, true},

    Target{0.0, 0.0, 0.0, 0.0},
  });

  return routes;
}

void RouteTestNode::loadRoute(RouteId route_id, const std::vector<Target> & route)
{
  route_locked_ = true;

  RCLCPP_INFO(
    get_logger(),
    "Received /route_choice=%u. Loading route with %zu targets.",
    static_cast<unsigned>(route_id),
    route.size());

  for (std::size_t index = 0; index < route.size(); ++index) {
    const auto & target = route[index];
    route_node_->addTarget(target);
    RCLCPP_INFO(
      get_logger(),
      "Loaded route %u target %zu/%zu: x=%.1f y=%.1f z=%.1f yaw=%.1f spray=%s",
      static_cast<unsigned>(route_id),
      index + 1,
      route.size(),
      target.x_cm,
      target.y_cm,
      target.z_cm,
      target.yaw_deg,
      target.spray ? "true" : "false");
  }

  const auto current = route_node_->currentIndex();
  RCLCPP_INFO(
    get_logger(),
    "Route %u is now active. Current target index=%zu",
    static_cast<unsigned>(route_id),
    (current == std::numeric_limits<std::size_t>::max() ? 0 : current + 1));
}

}  // namespace activity_control_pkg
