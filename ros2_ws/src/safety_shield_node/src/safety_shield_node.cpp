// include general headers
#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

// include ros2 headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

// include safety shield library
#include "safety_shield/safety_shield.h"
#include "point.hpp"

using namespace std::chrono_literals;

class SafetyShieldNode : public rclcpp::Node {
public:
  SafetyShieldNode()
  : Node("safety_shield_node"),
    t_(0.0),
    t_max_(10.0)
  {
    declareParameters();
    loadParameters();
    loadConfigFile();
    initializePublishers();
    initializeSubscribers();
    initializeShield();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(sample_time_),
      std::bind(&SafetyShieldNode::onTimer, this)
    );

    RCLCPP_INFO(this->get_logger(), "Safety Shield Node initialized");
  }

private:
  void declareParameters() {
    // system parameters
    // if the robot position should be synced to real system upon start
    this->declare_parameter<bool>("sync_robot_position", false);
    this->declare_parameter<double>("sample_time", 0.001);

    // Declare Safety Shield parameters
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
    
    // Environment elements
    this->declare_parameter<std::vector<double>>("environment.table.min", {-1.0, -1.0, -0.1});
    this->declare_parameter<std::vector<double>>("environment.table.max", {1.0, 1.0, 0.0});
    
    // Timeout for human measurements
    this->declare_parameter<double>("human_measurement_timeout", 2.0);
  }

  void loadParameters() {
    // Load system parameters
    robot_state_synchronized_ = true;
    bool sync_robot_position = false;
    // Try to get parameter; if missing, keep default true
    if (this->get_parameter("sync_robot_position", sync_robot_position)) {
      // If parameter exists, robot_state_synchronized_ is opposite of sync_robot_position
      robot_state_synchronized_ = !sync_robot_position;
    } else {
      // Parameter missing: warn and keep robot_state_synchronized_ = true
      RCLCPP_WARN(this->get_logger(), "Parameter 'sync_robot_position' not set. Using default 'true' for robot_state_synchronized_");
    }

    if (!this->get_parameter("sample_time", sample_time_)) {
      RCLCPP_WARN(this->get_logger(), "Parameter 'sample_time' not set. Using default 0.001.");
      sample_time_ = 0.001;
    }
    // Load Safety Shield parameters
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

    // Load shield type
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

    // Load environment elements
    std::vector<double> table_min, table_max;
    this->get_parameter("environment.table.min", table_min);
    this->get_parameter("environment.table.max", table_max);
    
    if (table_min.size() != 3 || table_max.size() != 3) {
      throw std::runtime_error("Table bounds must have 3 dimensions");
    }
    reach_lib::AABB table({table_min[0], table_min[1], table_min[2]}, 
                        {table_max[0], table_max[1], table_max[2]});
    environment_elements_.push_back(table);
  }

  void loadConfigFile() {
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
    if (robot_cfg["joint_names"]) {
      joint_names_ = robot_cfg["joint_names"].as<std::vector<std::string>>();
      if (joint_names_.size() != static_cast<size_t>(nb_joints)) {
        RCLCPP_ERROR(this->get_logger(), "Mismatch in number of joint names (%zu) and nb_joints (%d)",
                      joint_names_.size(), nb_joints);
        throw std::runtime_error("Mismatch in joint names count");
      }
    } else {
      // Default joint names if not provided
      joint_names_.resize(nb_joints);
      for (int i = 0; i < nb_joints; ++i) {
        joint_names_[i] = "joint" + std::to_string(i + 1);
      }
    }
  }

  void initializePublishers() {
    // Publishers
    current_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "current_joint_states", 10);
    desired_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "desired_joint_states", 10);
    safety_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "safety_flag", 10);
    human_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "human_reach_markers", 10);
    robot_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "robot_reach_markers", 10);
  }

  void initializeSubscribers() {
    // Subscribers
    human_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/human_measurements", 10,
      std::bind(&SafetyShieldNode::humanMeasurementCallback, this, std::placeholders::_1)
    );
    goal_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "goal_joint_states", 10,
      std::bind(&SafetyShieldNode::goalCallback, this, std::placeholders::_1));
    goal_trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "goal_trajectory_joint_states", 10,
      std::bind(&SafetyShieldNode::goalTrajectoryCallback, this, std::placeholders::_1));
    measured_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&SafetyShieldNode::initialJointStateCallback, this, std::placeholders::_1)
    );
  }

  void initializeShield() {
    new_goal_ = init_qpos_;

    shield_ = std::make_unique<safety_shield::SafetyShield>(
      sample_time_, trajectory_config_file_, robot_config_file_, mocap_config_file_,
      init_x_, init_y_, init_z_, init_roll_, init_pitch_, init_yaw_, init_qpos_,
      environment_elements_, shield_type_
    );
    //shield_->setNonPathConsistent();
  }

  void publishTimedCapsules(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
                            const std::vector<std::vector<std::vector<double>>>& capsules_over_time) {
    if (capsules_over_time.empty() || capsules_over_time[0].empty()) {
      RCLCPP_WARN(rclcpp::get_logger("SafetyShield"), "No capsule data to publish.");
      return;
    }

    // Take the first capsule from the first timestep
    std::vector<std::vector<double>> first_capsule_batch = capsules_over_time[0];

    // Publish with any fixed color_type (e.g., 0 = robot)
    publishCapsules(pub, first_capsule_batch, 0);
  }

  // Publishes reach capsules as MarkerArray via the given publisher
  void publishCapsules(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
                       const std::vector<std::vector<double>>& caps, int color_type) {
                       static visualization_msgs::msg::MarkerArray arr;
    arr.markers.clear();

    size_t id = 0;
    for (const auto &c : caps) {
      // Create two spheres at capsule endpoints
      auto sphere1 = makeSphere(c[0], c[1], c[2], c[6], id++, color_type);
      auto sphere2 = makeSphere(c[3], c[4], c[5], c[6], id++, color_type);
      // Create cylinder between endpoints
      auto cylinder = makeCylinder(c, id++, color_type);
      arr.markers.push_back(sphere1);
      arr.markers.push_back(sphere2);
      arr.markers.push_back(cylinder);
    }
    pub->publish(arr);
  }

  // Helper to construct a sphere marker
  visualization_msgs::msg::Marker makeSphere(double x, double y, double z,
                                             double radius, size_t id, int color_type) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = this->get_clock()->now();
    m.ns = "capsules";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.scale.x = 2.0 * radius;
    m.scale.y = 2.0 * radius;
    m.scale.z = 2.0 * radius;
    setColor(m, color_type);
    return m;
  }

  // Helper to construct a cylinder marker between two points
  visualization_msgs::msg::Marker makeCylinder(const std::vector<double>& c, size_t id, int color_type) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = this->get_clock()->now();
    m.ns = "capsules";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    // Midpoint
    m.pose.position.x = (c[0] + c[3]) / 2.0;
    m.pose.position.y = (c[1] + c[4]) / 2.0;
    m.pose.position.z = (c[2] + c[5]) / 2.0;
    // Orientation: align cylinder along vector from p1 to p2
    Eigen::Vector3d v(c[3] - c[0], c[4] - c[1], c[5] - c[2]);
    double L = v.norm();
    if (L > 1e-6) {
      Eigen::Vector3d axis = Eigen::Vector3d::UnitZ().cross(v);
      axis.normalize();
      double angle = std::acos(v.dot(Eigen::Vector3d::UnitZ()) / L);
      m.pose.orientation.x = axis.x() * std::sin(angle / 2.0);
      m.pose.orientation.y = axis.y() * std::sin(angle / 2.0);
      m.pose.orientation.z = axis.z() * std::sin(angle / 2.0);
      m.pose.orientation.w = std::cos(angle / 2.0);
      m.scale.z = L;
    }
    m.scale.x = 2.0 * c[6];
    m.scale.y = 2.0 * c[6];
    setColor(m, color_type);
    return m;
  }

  // Helper to set marker color based on type
  void setColor(visualization_msgs::msg::Marker &m, int type) const {
    m.color.r = m.color.g = m.color.b = 0.0f; 
    switch(type) {
      case 0: // robot reach
        m.color.g = 1.0f;
        break;
      case 2: // human reach
        m.color.r = 1.0f;
        break;
      default:
        m.color.r = m.color.g = m.color.b = 0.5f;
    }
    m.color.a = 0.3f;
  }

  void initialJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (robot_state_synchronized_) return;

    if (msg->position.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty joint state. Skipping initialization...");
      return;
    }

    // Map from joint name to index in incoming message
    std::unordered_map<std::string, size_t> joint_index_map;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_index_map[msg->name[i]] = i;
    }

    // Resize init_qpos_ to expected size (arm joints count)
    init_qpos_.resize(joint_names_.size());

    // Extract only positions for joints in joint_names_ - avoid gripper
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto &joint_name = joint_names_[i];
      auto it = joint_index_map.find(joint_name);
      if (it != joint_index_map.end()) {
        size_t idx = it->second;
        if (idx < msg->position.size()) {
          init_qpos_[i] = msg->position[idx];
        } else {
          RCLCPP_WARN(this->get_logger(), "Position index %zu out of bounds for joint '%s'", idx, joint_name.c_str());
          init_qpos_[i] = 0.0;
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Joint '%s' not found in joint_states", joint_name.c_str());
        init_qpos_[i] = 0.0;
      }
    }

    shield_->reset(init_x_, init_y_, init_z_, init_roll_, init_pitch_, init_yaw_,
                  init_qpos_, t_, environment_elements_, shield_type_);
    robot_state_synchronized_ = true;
    RCLCPP_INFO(this->get_logger(), "Shield initialized with filtered joint state.");
  }

  void humanMeasurementCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Check if the incoming message is valid
    if (msg->data.size() % 3 != 0) {
      RCLCPP_WARN(this->get_logger(), "Invalid human measurement data size: %zu (not divisible by 3)", 
                  msg->data.size());
      return;
    }
    human_measurement_.clear();
    size_t N = msg->data.size() / 3;
    for (size_t i = 0; i < N; ++i) {
      double x = msg->data[3*i];
      double y = msg->data[3*i+1];
      double z = msg->data[3*i+2];
      
      // Check for NaN or Inf values
      if (std::isnan(x) || std::isnan(y) || std::isnan(z) || 
          std::isinf(x) || std::isinf(y) || std::isinf(z)) {
        RCLCPP_WARN(this->get_logger(), "Invalid human measurement point %zu: [%f, %f, %f]", 
                    i, x, y, z);
        continue;
      }
      
      human_measurement_.push_back(reach_lib::Point(x, y, z));
    }
    
    has_new_measurement_ = true;
  }

  void goalCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // check incoming goal for size
    if ((int)msg->position.size() != (int)new_goal_.size()) {
      RCLCPP_WARN(this->get_logger(), "Received goal size (%zu) != nb_joints (%zu)",
                  msg->position.size(), new_goal_.size());
      return;
    }
    // ToDo check for joint limits
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
    if (!robot_state_synchronized_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for initial joint state to initialize shield...");
      return;
    }
    t_ += sample_time_;
    if (t_ > t_max_) t_ = fmod(t_, t_max_);

    if (has_new_measurement_){
      shield_->humanMeasurement(human_measurement_, t_);
      has_new_measurement_ = false;
    }

    if (has_new_goal_) {
      // udapte the shield with the new goal
      std::vector<double> zero_vel(new_goal_.size(), 0.0);
      shield_->newLongTermTrajectory(new_goal_, zero_vel);
      has_new_goal_ = false;
    }

    // publish current joint states
    auto cur_motion = shield_->getCurrentMotion();
    current_state_msg_.header.stamp = this->get_clock()->now();
    current_state_msg_.name = joint_names_;
    current_state_msg_.position = cur_motion.getAngle();
    current_state_msg_.velocity = cur_motion.getVelocity();
    current_state_pub_->publish(current_state_msg_);

    // perform safety shield step
    safety_shield::Motion next_motion = shield_->step(t_);

    // Publish desired joint states
    desired_state_msg_.header.stamp = this->get_clock()->now();
    desired_state_msg_.name = joint_names_;
    desired_state_msg_.position = next_motion.getAngle();
    desired_state_msg_.velocity = next_motion.getVelocity();
    desired_joint_state_pub_->publish(desired_state_msg_);

    // Publish safety flag
    safety_flag_msg_.data = shield_->getSafety();
    safety_flag_pub_->publish(safety_flag_msg_);

    // Publish human and robot capsules
    publishCapsules(human_marker_pub_, shield_->getHumanReachCapsules(0), 2);
    publishTimedCapsules(robot_marker_pub_, shield_->getAllRobotReachCapsulesOverTime());

  }

  // Preallocated messages
  sensor_msgs::msg::JointState current_state_msg_;
  sensor_msgs::msg::JointState desired_state_msg_;
  std_msgs::msg::Bool safety_flag_msg_;

  // Node components
  double t_{0.0}, t_max_{10.0};
  double sample_time_;
  double init_x_, init_y_, init_z_, init_roll_, init_pitch_, init_yaw_; 
  std::vector<double> init_qpos_, new_goal_;
  bool has_new_trajectory_{false};
  bool has_new_goal_{false};
  bool has_new_measurement_{false};
  bool robot_state_synchronized_;

  std::unique_ptr<safety_shield::SafetyShield> shield_;
  std::vector<reach_lib::AABB> environment_elements_;
  safety_shield::ShieldType shield_type_;
  std::vector<reach_lib::Point> human_measurement_;
  std::string trajectory_config_file_, robot_config_file_, mocap_config_file_;
  std::vector<std::string> joint_names_;

  // Pointers for Subscriptions and Publishers
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr human_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr goal_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr goal_trajectory_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measured_joint_state_sub_;


  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr current_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_flag_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr human_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr robot_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shield_mode_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyShieldNode>());
  rclcpp::shutdown();
  return 0;
}
