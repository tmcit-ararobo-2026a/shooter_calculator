// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: Apache-2.0
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <nlohmann/json.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

#include "shooter_calculator/shooter_calculator.hpp"
#include "shooter_calculator/msg/aim_target.hpp"
#include "shooter_calculator/msg/shot_solution.hpp"

namespace shooter_calculator {
/** @brief Typed aiming and calibrated belt conversion; no direct actuator publishers. */
class ShooterCalculatorNode : public rclcpp::Node {
public:
  using Clock = std::chrono::steady_clock;
  /** @brief Configure an explicitly approved physical model; missing calibration remains invalid. */
  ShooterCalculatorNode() : Node("shooter_calculator") {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto file = declare_parameter<std::string>("calibration_file", "", descriptor);
    target_timeout_s_ = declare_parameter<double>("target_timeout_s", 0.15, descriptor);
    pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 0.5, descriptor);
    if (!std::isfinite(target_timeout_s_) || target_timeout_s_ <= 0 ||
      !std::isfinite(pose_timeout_s_) || pose_timeout_s_ <= 0)
    {
      throw std::invalid_argument("Target/pose deadlines must be positive");
    }
    if (!file.empty()) {
      load_calibration(file);
    }
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    solution_publisher_ = create_publisher<msg::ShotSolution>("/shooter/solution", rclcpp::QoS(1).durability_volatile());
    trajectory_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/trajectory_markers", 10);
    target_subscription_ = create_subscription<msg::AimTarget>("/auto_mode/aim_target",
      rclcpp::QoS(1).durability_volatile(),
      [this](msg::AimTarget::ConstSharedPtr target) { accept_target(*target); });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { calculate(); });
  }
private:
  /** @brief Read finite profile numbers without clamping missing/invalid values. */
  double number(const nlohmann::json & config, const char * key) const {
    const double value = config.at(key).get<double>();
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Calibration values must be finite");
    }
    return value;
  }
  /** @brief Require physical units and monotonic measured release-to-belt pairs. */
  void load_calibration(const std::string & file) {
    std::ifstream stream(file);
    if (!stream) {
      throw std::invalid_argument("Cannot open calibration profile");
    }
    nlohmann::json config;
    stream >> config;
    if (config.at("schema_version").get<int>() != 1) {
      throw std::invalid_argument("Unsupported calibration schema");
    }
    if (!config.at("approved").get<bool>()) {
      return;
    }
    calibration_id_ = config.at("calibration_id").get<std::string>();
    map_frame_ = config.at("map_frame").get<std::string>();
    robot_frame_ = config.at("robot_frame").get<std::string>();
    const auto offset = config.at("shooter_offset_m").get<std::vector<double>>();
    if (calibration_id_.empty() || map_frame_.empty() || robot_frame_.empty() ||
      map_frame_ == robot_frame_ || offset.size() != 3)
    {
      throw std::invalid_argument("Calibration ID, frames and measured shooter offset are required");
    }
    for (const auto value : offset) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("Shooter offset must be finite");
      }
    }
    offset_ = tf2::Vector3(offset[0], offset[1], offset[2]);
    angle_deg_ = number(config, "launch_angle_deg");
    mass_kg_ = number(config, "cloth_mass_kg");
    area_m2_ = number(config, "cloth_area_m2");
    drag_ = number(config, "drag_coefficient");
    height_tolerance_m_ = number(config, "max_height_error_m");
    if (angle_deg_ <= 0 || angle_deg_ >= 90 || mass_kg_ < 0.01 || area_m2_ < 0 ||
      drag_ < 0 || height_tolerance_m_ <= 0 ||
      !std::isfinite(0.5 * 1.225 * drag_ * area_m2_ / mass_kg_))
    {
      throw std::invalid_argument("Calibration outside the supported ballistic model domain");
    }
    if (config.value("allow_moving_targets", false)) {
      throw std::invalid_argument("Moving target compensation contract is not yet implemented");
    }
    for (const auto & pair : config.at("calibration_pairs")) {
      if (!pair.is_array() || pair.size() != 2) {
        throw std::invalid_argument("Calibration pairs must be [initial_speed_m_s,belt_speed_m_s]");
      }
      const double initial = pair[0].get<double>();
      const double belt = pair[1].get<double>();
      if (!std::isfinite(initial) || !std::isfinite(belt) || initial < 1 || initial > 20 || belt == 0 ||
        std::abs(belt) > std::numeric_limits<float>::max()) {
        throw std::invalid_argument("Calibration speeds are invalid or outside solver range [1,20]");
      }
      if (!calibration_pairs_.empty()) {
        const auto & previous = calibration_pairs_.back();
        if (initial <= previous.first || std::abs(belt) <= std::abs(previous.second) ||
          (belt > 0) != (previous.second > 0))
        {
          throw std::invalid_argument("Calibration must increase initial speed and belt magnitude with one sign");
        }
      }
      calibration_pairs_.emplace_back(initial, belt);
    }
    if (calibration_pairs_.size() < 2) {
      throw std::invalid_argument("At least two approved calibration pairs are required");
    }
    calibrated_ = true;
  }
  /** @brief Admit complete current targets and reject stale sessions/generations/stamps. */
  void accept_target(const msg::AimTarget & target) {
    if (target.producer_session == 0 || target.generation == 0 || target.header.stamp.sec < 0 ||
      target.header.stamp.nanosec >= 1000000000U || retired_sessions_.count(target.producer_session) != 0)
    {
      return;
    }
    const auto stamp_ns = rclcpp::Time(target.header.stamp).nanoseconds();
    const double age_s = static_cast<double>(now().nanoseconds() - stamp_ns) * 1e-9;
    if (stamp_ns <= 0 || age_s < 0 || age_s >= target_timeout_s_) {
      return;
    }
    if (target.producer_session == target_.producer_session) {
      if (target.generation < target_.generation || stamp_ns <= rclcpp::Time(target_.header.stamp).nanoseconds()) {
        return;
      }
    } else if (target_.producer_session != 0) {
      retired_sessions_.insert(target_.producer_session);
    }
    target_ = target;
    have_target_ = true;
    target_deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(target_timeout_s_ - age_s));
  }
  /** @brief Verify actual model height at range; solver fallback alone is never a valid result. */
  bool height_at_range(double speed, double range, double & height,
    std::vector<geometry_msgs::msg::Point> & trajectory) const
  {
    const double angle_rad = angle_deg_ * std::acos(-1.0) / 180;
    double x = 0, z = 0, vx = speed * std::cos(angle_rad), vz = speed * std::sin(angle_rad);
    const double drag_factor = 0.5 * 1.225 * drag_ * area_m2_ / mass_kg_;
    constexpr double TIME_STEP_S = 0.005;
    for (int step = 0; step < 1000; ++step) {
      const double previous_x = x, previous_z = z;
      const double magnitude = std::hypot(vx, vz);
      vx -= drag_factor * magnitude * vx * TIME_STEP_S;
      vz += (-calculator_.GRAVITY - drag_factor * magnitude * vz) * TIME_STEP_S;
      x += vx * TIME_STEP_S;
      z += vz * TIME_STEP_S;
      if (!std::isfinite(x) || !std::isfinite(z) || vx <= 0) {
        return false;
      }
      geometry_msgs::msg::Point point;
      point.x = x;
      point.z = z;
      trajectory.push_back(point);
      if (x >= range) {
        const double ratio = (range - previous_x) / (x - previous_x);
        height = previous_z + ratio * (z - previous_z);
        return std::isfinite(height);
      }
    }
    return false;
  }
  /** @brief Interpolate only inside the explicitly measured calibration interval. */
  bool belt_speed(double initial_speed, double & belt) const {
    if (initial_speed < calibration_pairs_.front().first || initial_speed > calibration_pairs_.back().first) {
      return false;
    }
    for (std::size_t pair_index = 1; pair_index < calibration_pairs_.size(); ++pair_index) {
      const auto & first = calibration_pairs_[pair_index - 1];
      const auto & second = calibration_pairs_[pair_index];
      if (initial_speed <= second.first) {
        const double ratio = (initial_speed - first.first) / (second.first - first.first);
        belt = first.second + ratio * (second.second - first.second);
        return std::isfinite(belt);
      }
    }
    return false;
  }
  /** @brief Build a fresh invalid-or-calibrated solution and discard every old valid output. */
  void calculate() {
    msg::ShotSolution solution;
    solution.header.stamp = now();
    solution.header.frame_id = map_frame_;
    solution.producer_session = target_.producer_session;
    solution.generation = target_.generation;
    solution.target_id = target_.target_id;
    solution.calibration_id = calibration_id_;
    solution.reason = "No current AimTarget";
    const auto current_ns = now().nanoseconds();
    if (last_ros_now_ns_ > 0 && current_ns < last_ros_now_ns_) {
      have_target_ = false;
      last_pose_stamp_ns_ = 0;
    }
    last_ros_now_ns_ = current_ns;
    const double target_age_s = static_cast<double>(current_ns - rclcpp::Time(target_.header.stamp).nanoseconds()) * 1e-9;
    std::vector<geometry_msgs::msg::Point> trajectory;
    tf2::Vector3 origin;
    double direction_x = 0, direction_y = 0;
    try {
      if (!have_target_ || Clock::now() >= target_deadline_ || target_age_s < 0 || target_age_s >= target_timeout_s_) {
        throw std::runtime_error("AimTarget missing or expired");
      }
      if (!target_.valid) {
        throw std::runtime_error("AimTarget invalid");
      }
      if (!calibrated_) {
        throw std::runtime_error("Calibration missing or unapproved");
      }
      if (target_.target_id == 0 || target_.target_id > 7 || target_.header.frame_id != map_frame_) {
        throw std::runtime_error("Unsupported target ID/frame");
      }
      if (target_.target_id == 7) {
        throw std::runtime_error("Moving target compensation is not yet approved");
      }
      for (const auto value : {target_.position.x, target_.position.y, target_.position.z,
          target_.velocity_m_s.x, target_.velocity_m_s.y, target_.velocity_m_s.z, target_.confidence})
      {
        if (!std::isfinite(value)) {
          throw std::runtime_error("Target contains nonfinite fields");
        }
      }
      for (const auto value : target_.position_covariance) {
        if (!std::isfinite(value)) {
          throw std::runtime_error("Target covariance contains nonfinite values");
        }
      }
      for (std::size_t axis = 0; axis < 3; ++axis) {
        if (target_.position_covariance[axis * 3 + axis] < 0) {
          throw std::runtime_error("Target covariance diagonal is negative");
        }
      }
      if (target_.predicted || target_.confidence < 0 || target_.confidence > 1 ||
        target_.velocity_m_s.x != 0 || target_.velocity_m_s.y != 0 || target_.velocity_m_s.z != 0)
      {
        throw std::runtime_error("Static solution requires a valid stationary target");
      }
      const auto pose = buffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
      const double pose_age_s = (now() - rclcpp::Time(pose.header.stamp)).seconds();
      const auto pose_stamp_ns = rclcpp::Time(pose.header.stamp).nanoseconds();
      if (!std::isfinite(pose_age_s) || pose_age_s < 0 || pose_age_s >= pose_timeout_s_) {
        throw std::runtime_error("Robot pose expired");
      }
      if (pose_stamp_ns > last_pose_stamp_ns_) {
        last_pose_stamp_ns_ = pose_stamp_ns;
        pose_deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(pose_timeout_s_ - pose_age_s));
      }
      if (Clock::now() >= pose_deadline_) {
        throw std::runtime_error("Robot pose no longer advances");
      }
      const auto & rotation = pose.transform.rotation;
      const double norm = rotation.x * rotation.x + rotation.y * rotation.y +
                          rotation.z * rotation.z + rotation.w * rotation.w;
      if (!std::isfinite(norm) || std::abs(norm - 1) >= 1e-3 ||
        std::abs(rotation.x) > 1e-6 || std::abs(rotation.y) > 1e-6) {
        throw std::runtime_error("Robot pose quaternion invalid");
      }
      const tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
      const auto & position = pose.transform.translation;
      origin = tf2::Vector3(position.x, position.y, position.z) + tf2::quatRotate(quaternion, offset_);
      const double delta_x = target_.position.x - origin.x();
      const double delta_y = target_.position.y - origin.y();
      const double height_delta = target_.position.z - origin.z();
      const double range = std::hypot(delta_x, delta_y);
      if (!std::isfinite(range) || range <= 0 || !std::isfinite(height_delta)) {
        throw std::runtime_error("Target geometry is unreachable or invalid");
      }
      const double initial_speed = calculator_.getRequiredVelocityWithDrag(
        range, height_delta, angle_deg_, mass_kg_, area_m2_, drag_);
      double simulated_height = 0, belt = 0;
      if (!std::isfinite(initial_speed) || !height_at_range(initial_speed, range, simulated_height, trajectory) ||
        std::abs(simulated_height - height_delta) > height_tolerance_m_)
      {
        throw std::runtime_error("Target unreachable in the configured ballistic model");
      }
      if (!belt_speed(initial_speed, belt)) {
        throw std::runtime_error("Required initial speed outside measured calibration range");
      }
      solution.initial_speed_m_s = initial_speed;
      solution.belt_speed_m_s = belt;
      solution.in_calibration_range = true;
      solution.valid = true;
      solution.reason = "Valid static calibrated model solution";
      direction_x = delta_x / range;
      direction_y = delta_y / range;
    } catch (const std::exception & error) {
      solution.reason = error.what();
    }
    solution_publisher_->publish(solution);
    if (trajectory_publisher_->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker line;
      line.header = solution.header;
      line.ns = "shooter";
      line.id = 2;
      line.action = visualization_msgs::msg::Marker::DELETE;
      if (solution.valid) {
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.w = 1;
        line.scale.x = 0.025;
        line.color.g = 1;
        line.color.a = 1;
        for (const auto & sample : trajectory) {
          geometry_msgs::msg::Point point;
          point.x = origin.x() + direction_x * sample.x;
          point.y = origin.y() + direction_y * sample.x;
          point.z = origin.z() + sample.z;
          line.points.push_back(point);
        }
      }
      markers.markers.push_back(line);
      trajectory_publisher_->publish(markers);
    }
  }
  ShooterCalculator calculator_;
  bool calibrated_{}, have_target_{};
  std::string calibration_id_, map_frame_{"map"}, robot_frame_{"base_link"};
  double angle_deg_{}, mass_kg_{}, area_m2_{}, drag_{}, height_tolerance_m_{};
  double target_timeout_s_{}, pose_timeout_s_{};
  tf2::Vector3 offset_;
  msg::AimTarget target_{};
  int64_t last_ros_now_ns_{}, last_pose_stamp_ns_{};
  Clock::time_point target_deadline_{}, pose_deadline_{};
  std::vector<std::pair<double, double>> calibration_pairs_;
  std::set<uint64_t> retired_sessions_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<msg::ShotSolution>::SharedPtr solution_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_publisher_;
  rclcpp::Subscription<msg::AimTarget>::SharedPtr target_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace shooter_calculator

/** @brief Run the typed solver, which never publishes belt initialization or direct commands. */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<shooter_calculator::ShooterCalculatorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("shooter_calculator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
