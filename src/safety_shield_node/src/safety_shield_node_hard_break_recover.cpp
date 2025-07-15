// include general headers
#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

// include ruckig library
#include <ruckig/ruckig.hpp>

// include ros2 headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

// include safety shield library
#include "safety_shield/safety_shield.h"
#include "point.hpp"
#include "safety_shield/robot_reach.h"
#include "safety_shield/config_utils.h"
#include "safety_shield/trajectory_utils.h"
#include "safety_shield/motion.h"
using namespace std::chrono_literals;

struct Trajectory {
  std::vector<std::vector<double>> pos;
  std::vector<std::vector<double>> vel;
  std::vector<std::vector<double>> acc;
};

class SafetyShieldHardStopNodeRecover : public rclcpp::Node {
  public:
    SafetyShieldHardStopNodeRecover()
    : Node("safety_shield_node_hard_break_recover"),
      sample_time_(0.001),
      t_(0.0),
      t_max_(10.0)
    {
      declareParameters();
      loadParameters();
      loadConfigFile();
      initializePublishers();
      initializeSubscribers();
      initializeShield();

      new_goal_ = init_qpos_; // intit with initial qpos
      current_pos_ = init_qpos_;
      current_vel_ = std::vector<double>(6, 0.0);
      current_acc_ = std::vector<double>(6, 0.0);
      desired_pos_ = current_pos_;
      desired_vel_ = current_vel_;
      desired_acc_ = current_acc_;

      Trajectory initial_profile;
      initial_profile.pos.push_back(init_qpos_);
      initial_profile.vel.push_back(std::vector<double>(6, 0.0));
      initial_profile.acc.push_back(std::vector<double>(6, 0.0));
      ltt_ = initial_profile;
      current_traj_ = initial_profile; // initialize stopping trajectory
      trajectory_index_ = 0;
      ltt_index_ = 0;

      total_replan_attempts_ = 0;

      timer_ = this->create_wall_timer(
        std::chrono::duration<double>(sample_time_),
        std::bind(&SafetyShieldHardStopNodeRecover::onTimer, this)
      );

      RCLCPP_INFO(this->get_logger(), "Safety Shield Node initialized");
    }

  private:
    void declareParameters() {
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
      this->declare_parameter<std::vector<double>>("table.min", {-1.0, -1.0, -0.1});
      this->declare_parameter<std::vector<double>>("table.max", {1.0, 1.0, 0.0});
      
      // Timeout for human measurements
      this->declare_parameter<double>("human_measurement_timeout", 2.0);
    }

    void loadParameters() {
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
      this->get_parameter("table.min", table_min);
      this->get_parameter("table.max", table_max);
      
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
       // Joint names — for rviz display
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
        std::bind(&SafetyShieldHardStopNodeRecover::humanMeasurementCallback, this, std::placeholders::_1)
      );
      goal_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "goal_joint_states", 10,
        std::bind(&SafetyShieldHardStopNodeRecover::goalPlanningRuckigCallback, this, std::placeholders::_1));
    }

    void initializeShield() {
      new_goal_ = init_qpos_;

      shield_ = std::make_unique<safety_shield::SafetyShield>(
        sample_time_, trajectory_config_file_, robot_config_file_, mocap_config_file_,
        init_x_, init_y_, init_z_, init_roll_, init_pitch_, init_yaw_, init_qpos_,
        environment_elements_, shield_type_
      );
    }

    bool stepHardBreak(const std::vector<double>& start_q, const std::vector<double> start_dq,
                                                const std::vector<double> start_ddq) {
        if (start_q.size() < 6 || start_dq.size() < 6 || start_ddq.size() < 6) {
        RCLCPP_ERROR(get_logger(), "stepHardBreak: input vectors too small. q=%zu, dq=%zu, ddq=%zu",
                    start_q.size(), start_dq.size(), start_ddq.size());
        return false;
        }
        auto verify_start_time = std::chrono::high_resolution_clock::now();
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

        //Trajectory test_profile;
        // test if we can just calculate time points and take the trajectory from there with same steps
        //std::vector<double> time_points = safety_shield::calcTimePointsForEquidistantIntervals(0, sample_time_ * num_samples, sample_time_*5);
        // take trajectory at time points
        
        //int n_time_steps_ = time_points.size();
        // test_profile.pos.reserve(n_time_steps_);
        // test_profile.vel.reserve(n_time_steps_);
        // test_profile.acc.reserve(n_time_steps_);
        // // fill the trajectory at time points 
        // test_profile.pos.emplace_back(current_pos_);
        // test_profile.vel.emplace_back(current_vel_);
        // test_profile.acc.emplace_back(current_acc_);
        // for (std::size_t i = 1; i < time_points.size(); ++i) {
        //     double t1 = time_points[i]-sample_time_; // add sample time as stopping starts at 0.001
        //     out.trajectory.at_time(t1, qt, dqt, ddqt, sec);
        //     test_profile.pos.emplace_back(qt.begin(), qt.end());
        //     test_profile.vel.emplace_back(dqt.begin(), dqt.end());
        //     test_profile.acc.emplace_back(ddqt.begin(), ddqt.end());
        // }

        // std::vector<safety_shield::Motion> new_traj(num_samples + 1);
        // new_traj[0] = safety_shield::Motion(0.0, current_pos_, current_vel_, current_acc_, 0.0);
        // std::array<double, 6> qt{}, dqt{}, ddqt{};
        // size_t sec = 0; 
        // for (std::size_t i = 1; i < time_points.size(); ++i) {
        //   double t1 = time_points[i]-sample_time_; // add sample time as stopping starts at 0.001
        //   out.trajectory.at_time(t1, qt, dqt, ddqt, sec);
        //   // Convert std::array to std::vector
        //   std::vector<double> q(qt.begin(), qt.end());
        //   std::vector<double> dq(dqt.begin(), dqt.end());
        //   std::vector<double> ddq(ddqt.begin(), ddqt.end());
        //   std::vector<double> dddq(6, 0.0);  // Placeholder
        //   new_traj[i] = safety_shield::Motion(t1, q, dq, ddq, dddq);
        // }
        //bool ok1 = shield_->verify_hard_stop(new_traj, t_);

        // RCLCPP_INFO(this->get_logger(), "Time Points and 6-DOF Motions in ROS2:");

        // for (std::size_t i = 0; i < new_traj.size() && i < time_points.size(); ++i) {
        //     RCLCPP_INFO(this->get_logger(), "t = %.6f s", time_points[i]);

        //     const auto& angles = new_traj[i].getAngle();
        //     const auto& velocities = new_traj[i].getVelocity();
        //     const auto& accelerations = new_traj[i].getAcceleration();

        //     for (std::size_t j = 0; j < 6; ++j) {
        //         RCLCPP_INFO(this->get_logger(),
        //             "  Joint %zu | angle: %.6f | velocity: %.6f | acceleration: %.6f",
        //             j + 1, angles[j], velocities[j], accelerations[j]);
        //     }
        // }

                // print trajectory with time points
        // RCLCPP_INFO(get_logger(), "Trajectory at time points:");
        // for (size_t i = 0; i < test_profile.pos.size(); ++i) {
        //     RCLCPP_INFO(get_logger(), "t=%.3f, pos=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f], "
        //                              "vel=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f], "
        //                              "acc=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
        //                 time_points[i], 
        //                 test_profile.pos[i][0], test_profile.pos[i][1], test_profile.pos[i][2],
        //                 test_profile.pos[i][3], test_profile.pos[i][4], test_profile.pos[i][5],
        //                 test_profile.vel[i][0], test_profile.vel[i][1], test_profile.vel[i][2],
        //                 test_profile.vel[i][3], test_profile.vel[i][4], test_profile.vel[i][5],
        //                 test_profile.acc[i][0], test_profile.acc[i][1], test_profile.acc[i][2],
        //                 test_profile.acc[i][3], test_profile.acc[i][4], test_profile.acc[i][5]);
        // }

        bool ok = shield_->verify_trajectory(profile.pos, // q
                                            profile.vel, // qd
                                            profile.acc, // qdd
                                            t_);
        
        auto verify_end_time = std::chrono::high_resolution_clock::now();
        auto verify_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            verify_end_time - verify_start_time);
        
        //Log timing information
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
  
    bool replanTrajectoryWithRuckig(const std::vector<double>& start_q,
                                  const std::vector<double>& start_dq,
                                  const std::vector<double>& start_ddq,
                                  const std::vector<double>& goal_q,
                                  const std::vector<double>& goal_dq,
                                  const std::vector<double>& goal_ddq,
                                  double sample_time,
                                  Trajectory& traj) {
    constexpr size_t DOF = 6;

    std::array<double, DOF> q_arr{}, dq_arr{}, ddq_arr{};
    std::array<double, DOF> goal_q_arr{}, goal_dq_arr{}, goal_ddq_arr{};

    std::copy_n(start_q.begin(), DOF, q_arr.begin());
    std::copy_n(start_dq.begin(), DOF, dq_arr.begin());
    std::copy_n(start_ddq.begin(), DOF, ddq_arr.begin());
    std::copy_n(goal_q.begin(), DOF, goal_q_arr.begin());
    std::copy_n(goal_dq.begin(), DOF, goal_dq_arr.begin());
    std::copy_n(goal_ddq.begin(), DOF, goal_ddq_arr.begin());

    ruckig::Ruckig<DOF> otg(sample_time);
    ruckig::InputParameter<DOF> in;

    in.control_interface = ruckig::ControlInterface::Position;
    in.synchronization   = ruckig::Synchronization::Time;

    in.current_position     = q_arr;
    in.current_velocity     = dq_arr;
    in.current_acceleration = ddq_arr;

    in.target_position      = goal_q_arr;
    in.target_velocity      = goal_dq_arr;
    in.target_acceleration  = goal_ddq_arr;

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
      std::cerr << "[Ruckig] Trajectory duration invalid.\n";
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
    // std::ostringstream oss_goal;
    // oss_goal << "planned goal: [";
    // for (size_t i = 0; i < msg->position.size(); ++i) {
    //   oss_goal << msg->position[i] << (i+1<msg->position.size()? ", ": "]");
    // }
    // RCLCPP_INFO(this->get_logger(), "%s", oss_goal.str().c_str());

    // RCLCPP_INFO(rclcpp::get_logger("PlanningRuckig"),
    //   "Ruckig trajectory planned successfully, steps: %zu, duration: %.3f seconds",
    //   profile.pos.size(),
    //   profile.pos.size() * sample_time_);
    ltt_ = new_ltt_; // only add new trajectory if planning was successful
    ltt_index_ = 0;           // Reset trajectory execution index
    has_new_goal_ = true;
    new_goal_ = msg->position;      // Update new goal

    // RCLCPP_INFO(this->get_logger(), "Trajectory planned successfully with Ruckig.");
  }

  void onTimer() {
      using clock = std::chrono::steady_clock;
      auto cycle_time_start = clock::now();
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

      // Get current state from trajectory
      if (trajectory_index_ < current_traj_.pos.size()) {
          current_pos_ = current_traj_.pos[trajectory_index_];
          current_vel_ = current_traj_.vel[trajectory_index_];
          current_acc_ = current_traj_.acc[trajectory_index_];
      } else {
          // Once finished, hold the final state
          current_pos_ = current_traj_.pos.back();
          current_vel_ = current_traj_.vel.back();
          current_acc_ = current_traj_.acc.back();
          is_stopped_ = true;
      }

      // Publish current state
      current_state_msg_.header.stamp = this->get_clock()->now();
      current_state_msg_.name = joint_names_;
      current_state_msg_.position = current_pos_;
      current_state_msg_.velocity = current_vel_;
      current_state_pub_->publish(current_state_msg_);

      // Update shield with current state of human
      if (has_new_measurement_) {
          shield_->humanMeasurement(human_measurement_, t_);
          has_new_measurement_ = false;
      }

      // Handle new goal reception
      if (has_new_goal_) {
          // Debug prints
          std::ostringstream oss_curpos;
          oss_curpos << "Current position: [";
          for (size_t i = 0; i < current_pos_.size(); ++i) {
              oss_curpos << current_pos_[i] << (i+1<current_pos_.size()? ", ": "]");
          }
          RCLCPP_INFO(this->get_logger(), "%s", oss_curpos.str().c_str());

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
          total_replan_attempts_ = 0; // Reset counter for new goal
      }

      // Handle robot stopped state - immediate replanning attempt
      if (is_stopped_) {
          RCLCPP_INFO(this->get_logger(), "Robot is stopped, attempting to replan to goal.");
          
          std::ostringstream oss;
          oss << "Replanning to goal: [";
          for (size_t i = 0; i < new_goal_.size(); ++i) {
              oss << new_goal_[i] << (i+1<new_goal_.size()? ", ": "]");
          }
          RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

          Trajectory new_ltt_;
          bool success = planTrajectoryWithRuckig(
              current_pos_, current_vel_, current_acc_, new_goal_, sample_time_, new_ltt_);
          
          if (success) {
              ltt_ = new_ltt_;
              ltt_index_ = 0;
              is_stopped_ = false;
              on_breaking_trajectory_ = false;
              total_replan_attempts_ = 0;
              RCLCPP_INFO(this->get_logger(), "Successfully replanned trajectory from stopped state.");
          } else {
              RCLCPP_WARN(this->get_logger(), "Failed to replan from stopped state. Will retry.");
          }
      }

      // Main trajectory following logic
      if (!on_breaking_trajectory_) {
          // Normal trajectory following - check if next point is safe
          if (ltt_index_+1 < ltt_.pos.size()) {
              desired_pos_ = ltt_.pos[ltt_index_+1];
              desired_vel_ = ltt_.vel[ltt_index_+1];
              desired_acc_ = ltt_.acc[ltt_index_+1];
          } else {
              desired_pos_ = ltt_.pos.back();
              desired_vel_ = ltt_.vel.back();
              desired_acc_ = ltt_.acc.back();
          }

          using clock = std::chrono::steady_clock;
          auto start_time = clock::now();
          bool is_safe = stepHardBreak(desired_pos_, desired_vel_, desired_acc_);
          auto end_time = clock::now();
          auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

          if (!is_safe) {
              RCLCPP_ERROR(this->get_logger(), "Next point is unsafe, switching to breaking trajectory!");
              on_breaking_trajectory_ = true;
              total_replan_attempts_ = 0;
              
              // Follow the breaking trajectory
              if (trajectory_index_+1 < current_traj_.pos.size()) {
                  desired_pos_ = current_traj_.pos[trajectory_index_+1];
                  desired_vel_ = current_traj_.vel[trajectory_index_+1];
              } else {
                  desired_pos_ = current_traj_.pos.back();
                  desired_vel_ = current_traj_.vel.back();
              }
          }
      } else {
          // ON BREAKING TRAJECTORY - Continue following breaking trajectory while attempting to replan EVERY STEP
          
          // Follow the current breaking trajectory
          if (trajectory_index_+1 < current_traj_.pos.size()) {
              desired_pos_ = current_traj_.pos[trajectory_index_+1];
              desired_vel_ = current_traj_.vel[trajectory_index_+1];
          } else {
              // Breaking trajectory completed - robot will be stopped
              desired_pos_ = current_traj_.pos.back();
              desired_vel_ = current_traj_.vel.back();
          }

          // Attempt to replan back to the intended goal AT EVERY STEP
          total_replan_attempts_++;
          
          RCLCPP_DEBUG(this->get_logger(), 
              "Attempting to replan back to goal (total attempts: %d)", 
              total_replan_attempts_);

          Trajectory recovery_traj;
          bool replan_success = planTrajectoryWithRuckig(
              current_pos_, current_vel_, current_acc_, new_goal_, sample_time_, recovery_traj);
          
          if (replan_success) {
              // Check if the replanned trajectory is safe
              using clock = std::chrono::steady_clock;
              auto start_time = clock::now();
              bool is_recovery_safe = stepHardBreak(recovery_traj.pos[1], recovery_traj.vel[1], recovery_traj.acc[1]);
              auto end_time = clock::now();
              
              if (is_recovery_safe) {
                  // Successfully found a safe path back to the goal!
                  RCLCPP_INFO(this->get_logger(), 
                      "Successfully replanned safe trajectory back to goal after %d attempts!", 
                      total_replan_attempts_);
                  
                  ltt_ = recovery_traj;
                  ltt_index_ = 0;
                  on_breaking_trajectory_ = false;
                  total_replan_attempts_ = 0;
                  
                  // Update desired state to follow the new trajectory
                  desired_pos_ = ltt_.pos[1];
                  desired_vel_ = ltt_.vel[1];
                  desired_acc_ = ltt_.acc[1];
              } else {
                  RCLCPP_DEBUG(this->get_logger(), 
                      "Replanned trajectory is still unsafe, continuing with breaking trajectory");
              }
          } else {
              RCLCPP_DEBUG(this->get_logger(), 
                  "Failed to generate replan trajectory (attempt %d)", 
                  total_replan_attempts_);
          }
      }

      // Publish desired joint states
      desired_state_msg_.header.stamp = this->get_clock()->now();
      desired_state_msg_.name = joint_names_;
      desired_state_msg_.position = desired_pos_;
      desired_state_msg_.velocity = desired_vel_;
      desired_joint_state_pub_->publish(desired_state_msg_);

      // Publish safety flag
      safety_flag_msg_.data = shield_->getSafety();
      safety_flag_pub_->publish(safety_flag_msg_);

      auto cycle_time_end = clock::now();
      auto duration_cycle = std::chrono::duration_cast<std::chrono::microseconds>(cycle_time_end - cycle_time_start).count();
      RCLCPP_INFO(this->get_logger(), 
          "Cycle time: %ld μs (%.3f ms)",
          duration_cycle, duration_cycle / 1000.0);
      // Publish human and robot capsules
      if (current_traj_.pos.size() < 2) {
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
  visualization_msgs::msg::Marker makeCylinder(
    const std::vector<double>& c,
    size_t id,
    int color_type)
  {
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
    m.color.a = 0.3f;
  }


  // Preallocated messages
  sensor_msgs::msg::JointState current_state_msg_;
  sensor_msgs::msg::JointState desired_state_msg_;
  std_msgs::msg::Bool safety_flag_msg_;

  // Node components
  double sample_time_{0.001}, t_{0.0}, t_max_{10.0};
  double init_x_, init_y_, init_z_, init_roll_, init_pitch_, init_yaw_; 
  std::vector<double> init_qpos_, new_goal_;
  bool has_new_goal_{false};
  bool has_new_measurement_{false};
  std::unique_ptr<safety_shield::SafetyShield> shield_;
  std::vector<reach_lib::AABB> environment_elements_;
  safety_shield::ShieldType shield_type_;
  std::vector<reach_lib::Point> human_measurement_;
  std::string trajectory_config_file_, robot_config_file_, mocap_config_file_;
  std::vector<std::string> joint_names_;


  std::vector<double> current_pos_;
  std::vector<double> current_vel_;
  std::vector<double> current_acc_;
  std::vector<double> desired_pos_;
  std::vector<double> desired_vel_;
  std::vector<double> desired_acc_;
  Trajectory ltt_;
  Trajectory current_traj_;
  std::vector<std::vector<double>> new_trajectory_;
  size_t trajectory_index_{0};
  size_t ltt_index_{0}; // index for ltt trajectory
  bool on_breaking_trajectory_{false}; // true if we are on breaking trajectory
  bool is_stopped_{false}; // true if robot is stopped
  int total_replan_attempts_ = 0;  // For logging/debugging purposes only


  // Pointers for Subscriptions and Publishers
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr human_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr goal_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr current_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_flag_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr human_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr robot_marker_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyShieldHardStopNodeRecover>());
  rclcpp::shutdown();
  return 0;
}
