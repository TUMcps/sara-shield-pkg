#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "safety_shield/safety_shield.h"
#include "point.hpp"

using namespace std::chrono_literals;

class SafetyShieldNode : public rclcpp::Node {
public:
  SafetyShieldNode()
  : Node("safety_shield_node"),
    sample_time_(0.001),
    t_(0.0),
    t_max_(10.0)
  {
    // === Declare ROS parameters ===
    this->declare_parameter<std::string>("trajectory_config");
    this->declare_parameter<std::string>("robot_config");
    this->declare_parameter<std::string>("mocap_config");
    this->declare_parameter<std::string>("shield_type");
    this->declare_parameter<double>("init.pose.x", 0.0);
    this->declare_parameter<double>("init.pose.y", 0.0);
    this->declare_parameter<double>("init.pose.z", 0.0);
    this->declare_parameter<double>("init.pose.roll", 0.0);
    this->declare_parameter<double>("init.pose.pitch", 0.0);
    this->declare_parameter<double>("init.pose.yaw", 0.0);
    this->declare_parameter<std::vector<double>>("init.qpos", std::vector<double>());

    // === Retrieve ROS parameters ===
    this->get_parameter("trajectory_config", trajectory_config_file_);
    this->get_parameter("robot_config",     robot_config_file_);
    this->get_parameter("mocap_config",     mocap_config_file_);
    std::string shield_type;
    if (!this->get_parameter("shield_type", shield_type)) {
      RCLCPP_ERROR(this->get_logger(), "Required parameter 'shield_type' is missing");
      throw std::runtime_error("Missing required parameter: shield_type");
    }
    RCLCPP_INFO(this->get_logger(), "Requested shield_type='%s'", shield_type.c_str());

    this->get_parameter("init.pose.x", init_x_);
    this->get_parameter("init.pose.y", init_y_);
    this->get_parameter("init.pose.z", init_z_);
    this->get_parameter("init.pose.roll",  init_roll_);
    this->get_parameter("init.pose.pitch", init_pitch_);
    this->get_parameter("init.pose.yaw",   init_yaw_);
    this->get_parameter("init.qpos", init_qpos_);

    // Load robot configuration to get joint count
    YAML::Node robot_cfg;
    try {
      robot_cfg = YAML::LoadFile(robot_config_file_);
    } catch (const YAML::BadFile &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load robot_config '%s'", robot_config_file_.c_str());
      throw;
    }
    if (!robot_cfg["nb_joints"]) {
      RCLCPP_ERROR(this->get_logger(), "Missing 'nb_joints' in robot_config");
      throw std::runtime_error("nb_joints not found in config");
    }
    int nb_joints = robot_cfg["nb_joints"].as<int>();
    if (nb_joints <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid 'nb_joints' (%d)", nb_joints);
      throw std::runtime_error("nb_joints must be > 0");
    }

    joint_names_.resize(nb_joints);
    // Default joint names (override via remap or parameter if needed)
    for (int i = 0; i < nb_joints; ++i) {
      joint_names_[i] = "joint" + std::to_string(i+1);
    }

    // === Map shield type string to enum ===
    if (shield_type == "PFL") {
      shield_type_ = safety_shield::ShieldType::PFL;
    } else if (shield_type == "SSM") {
      shield_type_ = safety_shield::ShieldType::SSM;
    } else if (shield_type == "OFF") {
      shield_type_ = safety_shield::ShieldType::OFF;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported shield_type '%s'", shield_type.c_str());
      throw std::runtime_error("Unsupported shield_type");
    }
    RCLCPP_INFO(this->get_logger(), "Using shield_type='%s'", shield_type.c_str());


    // Table AABB default using initial zeros
    reach_lib::AABB table({-1.0, -1.0, -0.1}, {+1.0, +1.0, 0.0});
    environment_elements_.push_back(table);

    // Instantiate shield
    shield_ = std::make_unique<safety_shield::SafetyShield>(
      sample_time_, trajectory_config_file_, robot_config_file_, mocap_config_file_,
      init_x_, init_y_, init_z_, init_roll_, init_pitch_, init_yaw_, init_qpos_,
      environment_elements_, shield_type_
    );

    // Subscribe to human measurements
    human_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "human_measurements", 10,
      std::bind(&SafetyShieldNode::humanMeasurementCallback, this, std::placeholders::_1)
    );



    new_goal_ = init_qpos_; // intit with initial qpos
    // ROS interfaces
    goal_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "goal_joint_states", 10,
      std::bind(&SafetyShieldNode::goalCallback, this, std::placeholders::_1));
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "desired_joint_states", 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(sample_time_),
      std::bind(&SafetyShieldNode::onTimer, this)
    );

    RCLCPP_INFO(this->get_logger(), "Safety Shield Node initialized");
  }

private:

  void humanMeasurementCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    human_measurement_.clear();
    size_t N = msg->data.size() / 3;
    for (size_t i = 0; i < N; ++i) {
      // Convert incoming data to reach_lib::Point
      double x = msg->data[3*i];
      double y = msg->data[3*i+1];
      double z = msg->data[3*i+2];
      human_measurement_.push_back(reach_lib::Point(x, y, z));
    }
  }

  void goalCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if ((int)msg->position.size() != (int)new_goal_.size()) {
      RCLCPP_WARN(this->get_logger(), "Received goal size (%zu) != nb_joints (%zu)",
                  msg->position.size(), new_goal_.size());
      return;
    }
    new_goal_ = msg->position;
    has_new_goal_ = true;
  }

  void onTimer() {
      // wait for first human measurement
    if (human_measurement_.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for human measurements...");
      return;
    }
    t_ += sample_time_;
    if (t_ > t_max_) t_ = fmod(t_, t_max_);

    shield_->humanMeasurement(human_measurement_, t_);
    if (has_new_goal_) {
      std::vector<double> zero_vel(new_goal_.size(), 0.0);
      shield_->newLongTermTrajectory(new_goal_, zero_vel);
      has_new_goal_ = false;
    }

    safety_shield::Motion next_motion = shield_->step(t_);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.name = joint_names_;
    msg.position = next_motion.getAngle();
    msg.velocity = next_motion.getVelocity();
    joint_state_pub_->publish(msg);
  }

  // Node components
  double sample_time_{0.001}, t_{0.0}, t_max_{10.0};
  double init_x_, init_y_, init_z_, init_roll_, init_pitch_, init_yaw_; 
  std::vector<double> init_qpos_, new_goal_;
  bool has_new_goal_{false};
  std::unique_ptr<safety_shield::SafetyShield> shield_;
  std::vector<reach_lib::AABB> environment_elements_;
  safety_shield::ShieldType shield_type_;
  std::vector<reach_lib::Point> human_measurement_;
  std::string trajectory_config_file_, robot_config_file_, mocap_config_file_;
  std::vector<std::string> joint_names_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr human_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr goal_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyShieldNode>());
  rclcpp::shutdown();
  return 0;
}
