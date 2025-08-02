#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>        
#include <moveit/robot_model/robot_model.h>                      
#include <moveit/robot_state/robot_state.h>                     
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <chrono>

class CartesianToJointConverter : public rclcpp::Node
{
public:
    CartesianToJointConverter() : Node("cartesian_to_joint_converter")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Cartesian to Joint Converter...");

        current_joint_positions_ = {-1.73960110e-02,
        9.55319758e-02,
        8.09703053e-04,
        -1.94272034e00,
        -4.01435784e-03,
        2.06584183e00,
        7.97426445e-01,};

        joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/goal_trajectory_joint_states", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/visualization_marker", 10);

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/target_poses", 10,
            std::bind(&CartesianToJointConverter::targetPosesCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/current_joint_states", 10,
            std::bind(&CartesianToJointConverter::jointStateCallback, this, std::placeholders::_1));


        RCLCPP_INFO(this->get_logger(), "Waiting for pose messages...");
    }

    void initialize()
    {
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
            shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader_->getModel();

        if (!robot_model_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model");
            return;
        }

        planning_group_ = "panda_arm";
        joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);

        if (!joint_model_group_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get joint model group: %s", planning_group_.c_str());
            return;
        }

        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        robot_state_->setToDefaultValues();
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_pose_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::shared_ptr<moveit::core::RobotState> robot_state_;
    std::vector<double> current_joint_positions_;
    std::string planning_group_;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() != 7) return;  // Expecting 7 DOF for panda_arm
        current_joint_positions_ = msg->position;
    }

    void publishPoseMarkers(const geometry_msgs::msg::PoseArray::SharedPtr& msg)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "target_poses";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01;  // point size
        marker.scale.y = 0.01;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker.lifetime = rclcpp::Duration::from_seconds(10.0);  // or 0 for forever

        for (const auto& pose : msg->poses)
        {
            geometry_msgs::msg::Point p;
            p.x = pose.position.x;
            p.y = pose.position.y;
            p.z = pose.position.z;
            marker.points.push_back(p);
        }

        marker_pub_->publish(marker);
    }

    void targetPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty pose array");
            return;
        }

        trajectory_msgs::msg::JointTrajectory joint_trajectory;
        if (processCartesianWaypoints(msg->poses, joint_trajectory))
        {
            joint_trajectory_pub_->publish(joint_trajectory);
            publishPoseMarkers(msg);
            // simulateJointTrajectory(joint_trajectory);
        }
    }

    bool cartesianToJoint(const geometry_msgs::msg::Pose& target_pose,
                          std::vector<double>& joint_positions)
    {
        robot_state_->setJointGroupPositions(joint_model_group_, current_joint_positions_);
        bool found_ik = robot_state_->setFromIK(joint_model_group_, target_pose, "panda_hand_tcp", 0.1);


        if (found_ik)
        {
            robot_state_->copyJointGroupPositions(joint_model_group_, joint_positions);
            return true;
        }
        return false;
    }

    bool processCartesianWaypoints(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                trajectory_msgs::msg::JointTrajectory& joint_trajectory)
    {
        RCLCPP_INFO(this->get_logger(), "Processing %zu waypoints", waypoints.size());

        auto start = std::chrono::steady_clock::now();

        joint_trajectory.header.stamp = this->now();
        joint_trajectory.header.frame_id = "world";
        joint_trajectory.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3",
                                        "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
        joint_trajectory.points.clear();

        double time_from_start = 0.0;
        const double dt = 0.1;

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            std::vector<double> joint_positions;
            if (!cartesianToJoint(waypoints[i], joint_positions))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to compute IK for pose %zu", i);
                return false;
            }

            // Add to trajectory
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions = joint_positions;
            pt.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
            joint_trajectory.points.push_back(pt);

            current_joint_positions_ = joint_positions;
            time_from_start += dt;

            // Immediately publish current joint state
            if (i == 0){
            sensor_msgs::msg::JointState js;
            js.header.stamp = this->now();
            js.name = joint_trajectory.joint_names;
            js.position = current_joint_positions_;
            joint_state_pub_->publish(js);
            }
        }

        auto end = std::chrono::steady_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        RCLCPP_INFO(this->get_logger(), "Finished computing %zu waypoints in %ld ms",
                    waypoints.size(), duration_ms);

        return true;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianToJointConverter>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
