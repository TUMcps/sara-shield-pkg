#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        self.waypoints = [
            [1.7, 1.40, 0.8557245716351515, -0.67643376135128,
             -0.15718006952603658, 3.358743019147018, 0.7184185826257936],
            [-0.47056755812951995, 0.9410700825585258, 1.1853199879239116,
             -0.6521574217310434, -0.3002404732369599,
             2.8922996513627623, 0.9543601782592442],
            [-0.75, 1.2242576980256197, 0.5952448330664808,
             -0.7961084507222761, -0.3055968793597486,
             2.6919983236856706, 0.8166998131787222],
            [-0.47056755812951995, 0.9410700825585258, 1.1853199879239116,
             -0.6521574217310434, -0.3002404732369599,
             2.8922996513627623, 0.9543601782592442],
        ]

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            'goal_trajectory_joint_states',
            10
        )

        self.timer = self.create_timer(0.5, self.publish_all_waypoints_once)

    def publish_all_waypoints_once(self):
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = [
            f'joint{i+1}' for i in range(len(self.waypoints[0]))
        ]

        for i, waypoint in enumerate(self.waypoints):
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.time_from_start.sec = (i + 1) * 3
            point.time_from_start.nanosec = 0
            traj_msg.points.append(point)

        self.traj_pub.publish(traj_msg)
        self.get_logger().info(
            f'→ Published all {len(self.waypoints)} waypoints as a single trajectory'
        )
        self.timer.cancel()


def main(args=None):  # ← must be at module level, NOT inside the class
    rclpy.init(args=args)
    node = WaypointPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()