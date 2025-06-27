#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <ruckig/ruckig.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "safety_shield/safety_shield.h"
#include "point.hpp"
#include "safety_shield/robot_reach.h"
#include "safety_shield/config_utils.h"
#include "safety_shield/trajectory_utils.h"

using namespace std::chrono_literals;

struct Trajectory {
  std::vector<std::vector<double>> pos;
  std::vector<std::vector<double>> vel;
  std::vector<std::vector<double>> acc;
};

class SafetyShieldHardStopNode : public rclcpp::Node {
public:
  SafetyShieldHardStopNode()
  : Node("safety_shield_node_hard_break"),
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
    if (init_qpos_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "init.qpos is empty! Make sure it's set in the parameters.");
    throw std::runtime_error("Missing init.qpos");
    }
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
#pragma omp parallel for
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

    // robot_reach_.reset(
    //     safety_shield::buildRobotReach( robot_config_file_,
    //                                     init_x_, init_y_, init_z_,
    //                                     init_roll_, init_pitch_, init_yaw_) );
    // Subscribe to human measurements
    human_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/human_measurements", 10,
      std::bind(&SafetyShieldHardStopNode::humanMeasurementCallback, this, std::placeholders::_1)
    );

    // ROS interfaces
    goal_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "goal_joint_states", 10,
      std::bind(&SafetyShieldHardStopNode::goalPlanningRuckigCallback, this, std::placeholders::_1));
    current_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "current_joint_states", 10);
    desired_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "desired_joint_states", 10);
    safety_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "safety_flag", 10);
    human_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "human_reach_markers", 10);
    robot_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "robot_reach_markers", 10);

    new_goal_ = init_qpos_; // intit with initial qpos
    current_pos_ = init_qpos_;
    current_vel_ = std::vector<double>(nb_joints, 0.0);
    current_acc_ = std::vector<double>(nb_joints, 0.0);
    desired_pos_ = current_pos_;
    desired_vel_ = current_vel_;
    desired_acc_ = current_acc_;

    Trajectory initial_profile;
    initial_profile.pos.push_back(init_qpos_);
    initial_profile.vel.push_back(std::vector<double>(nb_joints, 0.0));
    initial_profile.acc.push_back(std::vector<double>(nb_joints, 0.0));
    ltt_ = initial_profile;
    current_traj_ = initial_profile; // initialize stopping trajectory
    trajectory_index_ = 0;
    ltt_index_ = 0;

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(sample_time_),
      std::bind(&SafetyShieldHardStopNode::onTimer, this)
    );

    RCLCPP_INFO(this->get_logger(), "Safety Shield Node initialized");
  }

private:
    bool stepHardBreak(const std::vector<double>& start_q, const std::vector<double> start_dq,
                                                const std::vector<double> start_ddq) {
        if (start_q.size() < 6 || start_dq.size() < 6 || start_ddq.size() < 6) {
        RCLCPP_ERROR(get_logger(), "stepHardBreak: input vectors too small. q=%zu, dq=%zu, ddq=%zu",
                    start_q.size(), start_dq.size(), start_ddq.size());
        return false;
        }
        // convert to arr                                         
        std::array<double,6> q_arr, dq_arr, ddq_arr;
        Trajectory profile;  
        std::copy_n(start_q .begin(), 6, q_arr .begin());
        std::copy_n(start_dq.begin(), 6, dq_arr.begin());
        std::copy_n(start_ddq.begin(),6, ddq_arr.begin());

        ruckig::Ruckig<6> otg(sample_time_);
        // 2) Fill inputs for “brake to zero”
        ruckig::InputParameter<6> in;
        in.control_interface    = ruckig::ControlInterface::Velocity;
        in.synchronization      = ruckig::Synchronization::Time;
        in.current_position     = q_arr;
        in.current_velocity     = dq_arr;
        in.current_acceleration = ddq_arr;
        in.target_velocity.fill(0.0);
        in.target_acceleration.fill(0.0);
        in.max_velocity         = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        in.max_acceleration     = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
        in.max_jerk             = {400.0, 400.0, 400.0, 400.0, 400.0, 400.0};

        ruckig::OutputParameter<6> out;
        auto res = otg.update(in,out);
        if (res != ruckig::Result::Working && res != ruckig::Result::Finished)
        {
          RCLCPP_ERROR(get_logger(), "Ruckig error: %d", static_cast<int>(res));
          return false;
        }
        const double T = out.trajectory.get_duration();
        double new_time = 0.0;
        const size_t num_samples = static_cast<size_t>(std::ceil(T / sample_time_)) + 1;
        
        // 3) Create trajectory profile add current state
        profile.pos.reserve(num_samples+1);
        profile.vel.reserve(num_samples+1);
        profile.acc.reserve(num_samples+1);

        // check if motion to next point + brake to zero is safe
        profile.pos.emplace_back(current_pos_);
        profile.vel.emplace_back(current_vel_);
        profile.acc.emplace_back(current_acc_);

        profile.pos.emplace_back(start_q);
        profile.vel.emplace_back(start_dq);
        profile.acc.emplace_back(start_ddq);

        std::array<double,6> q{}, dq{}, ddq{};
        std::vector<double> times; // Add this to store time values
        times.push_back(0.0); // Initial time is 0.0
        size_t section = 0; // keeps track of poly segment

        for (size_t k = 1; k < num_samples; ++k) {
            double t = k * sample_time_;
            times.push_back(t); // Store the time
            out.trajectory.at_time(t, q, dq, ddq, section);
            profile.pos.emplace_back(q.begin(), q.end());
            profile.vel.emplace_back(dq.begin(), dq.end());
            profile.acc.emplace_back(ddq.begin(),ddq.end());
        }

        auto verify_start_time = std::chrono::high_resolution_clock::now();
        
        bool ok = shield_->verify_trajectory(profile.pos, // q
                                            profile.vel, // qd
                                            profile.acc, // qdd
                                            t_);
        
        auto verify_end_time = std::chrono::high_resolution_clock::now();
        auto verify_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            verify_end_time - verify_start_time);
        
        // Log timing information
        RCLCPP_INFO(get_logger(), 
                  "verify_trajectory took: %ld μs (%.3f ms) for %zu samples", 
                  verify_duration.count(), 
                  verify_duration.count() / 1000.0,
                  profile.pos.size());
        if (ok) {
            current_traj_ = profile;
            trajectory_index_ = 0; // Reset trajectory execution index
        }
         // store the stopping trajectory
        return ok;  
    }
  
    bool planTrajectoryWithRuckig(const std::vector<double>& start_q,
                                const std::vector<double>& start_dq,
                                const std::vector<double>& start_ddq,
                                const std::vector<double>& goal_q,
                                double sample_time, Trajectory& traj) {
    // check if the goal is the same as the current goal
    if (start_q == goal_q) {
      RCLCPP_INFO(this->get_logger(), "Received goal is the same as current goal, skipping planning.");
      return false;
    }
    constexpr size_t DOF = 6; // Adjust as necessary
    std::array<double, DOF> q_arr, dq_arr, ddq_arr, goal_arr;
                             
    std::copy_n(start_q.begin(), DOF, q_arr.begin());
    std::copy_n(start_dq.begin(), DOF, dq_arr.begin());
    std::copy_n(start_ddq.begin(), DOF, ddq_arr.begin());
    std::copy_n(goal_q.begin(), DOF, goal_arr.begin());

    ruckig::Ruckig<DOF> otg(sample_time);
    ruckig::InputParameter<DOF> in;

    in.control_interface    = ruckig::ControlInterface::Position;
    in.synchronization      = ruckig::Synchronization::Time;
    in.current_position     = q_arr;
    in.current_velocity     = dq_arr;
    in.current_acceleration = ddq_arr;
    in.target_position      = goal_arr;
    in.target_velocity.fill(0.0);
    in.target_acceleration.fill(0.0);
    in.max_velocity.fill(1.0);
    in.max_acceleration.fill(2.0);
    in.max_jerk.fill(15.0);

    ruckig::OutputParameter<DOF> out;
    bool was_interrupted = false;

    auto result = otg.calculate(in, out.trajectory, was_interrupted);


    if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
      return false;
    }

    if (out.trajectory.get_duration() <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Ruckig trajectory duration is zero or negative.");
      return false;
    }

    const double T = out.trajectory.get_duration();
    size_t num_samples = static_cast<size_t>(std::ceil(T / sample_time)) + 1;
    
    traj.pos.resize(num_samples, std::vector<double>(DOF));
    traj.vel.resize(num_samples, std::vector<double>(DOF));
    traj.acc.resize(num_samples, std::vector<double>(DOF));

    std::array<double, DOF> q{}, dq{}, ddq{};
    size_t section = 0;

    for (size_t k = 0; k < num_samples; ++k) {
      double t = k * sample_time;
      out.trajectory.at_time(t, q, dq, ddq, section);

      for (size_t i = 0; i < DOF; ++i) {
        traj.pos[k][i] = q[i];
        traj.vel[k][i] = dq[i];
        traj.acc[k][i] = ddq[i];
      }
    }

    return true;
  }
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
  void goalPlanningRuckigCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() != new_goal_.size()) {
      RCLCPP_WARN(this->get_logger(), "Received goal size (%zu) != nb_joints (%zu)",
                  msg->position.size(), new_goal_.size());
      return;
    }
    
    using clock = std::chrono::steady_clock;
    auto start_time = clock::now();
    Trajectory new_ltt_;
    bool success = planTrajectoryWithRuckig(
        current_pos_, current_vel_, current_acc_, msg->position, sample_time_,new_ltt_);

    auto end_time = clock::now();
    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    
    if (!success) {
      RCLCPP_ERROR(get_logger(), "Failed to plan trajectory with Ruckig.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), 
        "Trajectory planning completed in %ld µs (%zu steps)",
        duration_us, ltt_.pos.size());

    // print goal
    std::ostringstream oss_goal;
    oss_goal << "planned goal: [";
    for (size_t i = 0; i < msg->position.size(); ++i) {
      oss_goal << msg->position[i] << (i+1<msg->position.size()? ", ": "]");
    }
    RCLCPP_INFO(this->get_logger(), "%s", oss_goal.str().c_str());

    // RCLCPP_INFO(rclcpp::get_logger("PlanningRuckig"),
    //   "Ruckig trajectory planned successfully, steps: %zu, duration: %.3f seconds",
    //   profile.pos.size(),
    //   profile.pos.size() * sample_time_);
    ltt_ = new_ltt_; // only add new trajectory if planning was successful
    ltt_index_ = 0;           // Reset trajectory execution index
    has_new_goal_ = true;
    new_goal_ = msg->position;      // Update new goal

    RCLCPP_INFO(this->get_logger(), "Trajectory planned successfully with Ruckig.");
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

    ++trajectory_index_;
    ++ltt_index_;

    if (current_traj_.pos.empty() || current_traj_.vel.empty() || current_traj_.acc.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty current_profile detected, skipping timer step.");
    return;
    }
    // get current state from trajectory
    if (trajectory_index_ < current_traj_.pos.size()) {
    current_pos_ = current_traj_.pos[trajectory_index_];
    current_vel_ = current_traj_.vel[trajectory_index_];
    current_acc_ = current_traj_.acc[trajectory_index_];
    } else {
    // Once finished, hold the final state
    current_pos_ = current_traj_.pos.back();
    current_vel_ = current_traj_.vel.back();
    current_acc_ = current_traj_.acc.back();
    is_stopped_ = true; // Mark as stopped
    }

    // publish current state
    sensor_msgs::msg::JointState current_state_msg;
    current_state_msg.header.stamp = this->get_clock()->now();
    current_state_msg.name = joint_names_;
    current_state_msg.position = current_pos_;
    current_state_msg.velocity = current_vel_;
    current_state_pub_->publish(current_state_msg);

    // Update shield with current state of human
    shield_->humanMeasurement(human_measurement_, t_);

    // Check if we have a new goal - update ltt_ - currently planner only sends new goals if old goal is finished
    if (has_new_goal_) {
      // print current position for debugging
      std::ostringstream oss_curpos;
      oss_curpos << "Current position: [";
      for (size_t i = 0; i < current_pos_.size(); ++i) {
        oss_curpos << current_pos_[i] << (i+1<current_pos_.size()? ", ": "]");
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss_curpos.str().c_str());
      // print current velocity for debugging
      std::ostringstream oss_curvel;
      oss_curvel << "Current velocity: [";
      for (size_t i = 0; i < current_vel_.size(); ++i) {
        oss_curvel << current_vel_[i] << (i+1<current_vel_.size()? ", ": "]");
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss_curvel.str().c_str());


      std::ostringstream oss;
      oss << "New goal received: [";
      for (size_t i = 0; i < new_goal_.size(); ++i) {
        oss << new_goal_[i] << (i+1<new_goal_.size()? ", ": "]");
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
      has_new_goal_ = false;
    }
    if (is_stopped_) {
      RCLCPP_INFO(this->get_logger(), "Robot is stopped, replanning.");
      // Replan trajectory to the new goal
      Trajectory new_ltt_;
      // print new_goal_ for debugging
      std::ostringstream oss;
      oss << "Replanning to goal: [";
      for (size_t i = 0; i < new_goal_.size(); ++i) {
        oss << new_goal_[i] << (i+1<new_goal_.size()? ", ": "]");
      }
      RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

      bool success = planTrajectoryWithRuckig(
        current_pos_, current_vel_, current_acc_, new_goal_, sample_time_, new_ltt_);
      // if (!success) {
      //   RCLCPP_ERROR(get_logger(), "Failed to replan trajectory with Ruckig.");
      //   return;
      // }
      if (success){
        ltt_ = new_ltt_; // Update ltt_ with the new planned trajectory
        ltt_index_ = 0; // Reset trajectory execution index
        is_stopped_ = false; // Reset stopped state
        on_breaking_trajectory_ = false; // Reset breaking trajectory state
        RCLCPP_INFO(this->get_logger(), "Replanned trajectory successfully.");
      }

    }


    // logic from now on - verify next trajectory point if we find breaking trajectory - if not stop the robot - if safe just continue
    if (!on_breaking_trajectory_) {
      // get next desired state from trajectory
      if (ltt_index_+1 < ltt_.pos.size()) {
      desired_pos_ = ltt_.pos[ltt_index_+1];
      desired_vel_ = ltt_.vel[ltt_index_+1];
      desired_acc_ = ltt_.acc[ltt_index_+1];
      } else {
      // Once finished, hold the final state
      desired_pos_ = ltt_.pos.back();
      desired_vel_ = ltt_.vel.back();
      desired_acc_ = ltt_.acc.back();
      }
      using clock = std::chrono::steady_clock;
      auto start_time = clock::now();
      bool is_safe = stepHardBreak(desired_pos_, desired_vel_, desired_acc_);// if safe we overwrite current trajectory else we stop the robot
      auto end_time = clock::now();
      auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

      // RCLCPP_INFO(this->get_logger(), 
      //     "failsafe planning completed in %ld µs)",
      //     duration_us);

      if (!is_safe) {
        RCLCPP_ERROR(this->get_logger(), "Breaking trajectory is not safe, stopping robot!");
        on_breaking_trajectory_ = true;
        // move along breaking trajectory - follow current path to zero velocity
        if (trajectory_index_+1 < current_traj_.pos.size()) {
        desired_pos_ = current_traj_.pos[trajectory_index_+1];
        desired_vel_ = current_traj_.vel[trajectory_index_+1];
        //desired_acc_ = current_traj_.acc[trajectory_index_+1];
        } else {
        // Once finished, hold the final state
        desired_pos_ = current_traj_.pos.back();
        desired_vel_ = current_traj_.vel.back();
        //desired_acc_ = current_traj_.acc.back();
        }
      } 
      // else {
      //   RCLCPP_INFO(this->get_logger(), "Breaking trajectory is safe, continuing with next point.");
      // }
    }
    // Publish desired joint states
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.name = joint_names_;
    msg.position = desired_pos_;
    msg.velocity = desired_vel_;
    desired_state_pub_->publish(msg);

    // Publish safety flag
    std_msgs::msg::Bool safety_flag_msg;
    safety_flag_msg.data = shield_->getSafety();
    safety_flag_pub_->publish(safety_flag_msg);

    //Publish human and robot capsules
    if (current_traj_.pos.size()<2) {
      RCLCPP_WARN(this->get_logger(), "Stopping trajectory has less than 2 points, skipping capsule publishing.");
    } else {
        auto human_capsules = shield_->getHumanReachCapsules(0);
        auto robot_capsules = shield_->getRobotReachCapsules();
        publishCapsules(human_marker_pub_, human_capsules, 2);
        publishCapsules(robot_marker_pub_, robot_capsules, 0);
    }

  }
  // Publishes reach capsules as MarkerArray via the given publisher
  void publishCapsules(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub,
    const std::vector<std::vector<double>>& caps,
    int color_type)
  {
    visualization_msgs::msg::MarkerArray arr;
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
  visualization_msgs::msg::Marker makeSphere(
    double x, double y, double z,
    double radius,
    size_t id,
    int color_type)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
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
  visualization_msgs::msg::Marker makeCylinder(
    const std::vector<double>& c,
    size_t id,
    int color_type)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
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
  void setColor(visualization_msgs::msg::Marker &m, int type) {
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
    m.color.a = 0.8f;
  }
  // Node components
  double sample_time_{0.001}, t_{0.0}, t_max_{10.0};
  double init_x_, init_y_, init_z_, init_roll_, init_pitch_, init_yaw_; 
  std::vector<double> init_qpos_, new_goal_;
  std::vector<double> current_pos_;
  std::vector<double> current_vel_;
  std::vector<double> current_acc_;
  std::vector<double> desired_pos_;
  std::vector<double> desired_vel_;
  std::vector<double> desired_acc_;
  Trajectory ltt_;
  Trajectory current_traj_;

  bool has_new_goal_{false};
  bool on_breaking_trajectory_{false}; // true if we are on breaking trajectory
  bool is_stopped_{false}; // true if robot is stopped


  size_t trajectory_index_{0};
  size_t ltt_index_{0}; // index for ltt trajectory
  std::unique_ptr<safety_shield::SafetyShield> shield_;
  // std::unique_ptr<safety_shield::RobotReach> robot_reach_;
  std::vector<reach_lib::AABB> environment_elements_;
  safety_shield::ShieldType shield_type_;
  std::vector<reach_lib::Point> human_measurement_;
  std::string trajectory_config_file_, robot_config_file_, mocap_config_file_;
  std::vector<std::string> joint_names_;
  std::vector<std::vector<double>> new_trajectory_;


  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr human_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr goal_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr current_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_flag_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr human_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr robot_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyShieldHardStopNode>());
  rclcpp::shutdown();
  return 0;
}
