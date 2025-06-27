#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import os
import argparse

class StateLogger(Node):
    def __init__(self, filepath: str):
        super().__init__('state_logger')
        self.get_logger().info(f"StateLogger started. Writing to '{filepath}'")

        self.sub = self.create_subscription(
            JointState,
            '/current_joint_states',
            self.listener_callback,
            10
        )

        # Ensure the directory exists
        os.makedirs(os.path.dirname(filepath), exist_ok=True)

        # If an old log exists, rename it
        if os.path.exists(filepath):
            os.replace(filepath, filepath + '.old')

        # Open the CSV file for writing
        self.csv_file = open(filepath, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'name', 'position', 'velocity', 'effort'])

    def listener_callback(self, msg: JointState):
        now = self.get_clock().now().to_msg()
        t = now.sec + now.nanosec * 1e-9
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else ''
            vel = msg.velocity[i] if i < len(msg.velocity) else ''
            eff = msg.effort[i]   if i < len(msg.effort)   else ''
            self.csv_writer.writerow([t, name, pos, vel, eff])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    # Parse a command-line argument for the log filename
    parser = argparse.ArgumentParser(
        description='ROS2 node that logs /current_joint_states to a CSV.'
    )
    parser.add_argument(
        'filename',
        nargs='?',
        default='joint_states_log.csv',
        help='Name of the CSV file to write (will be placed under the specified utils/data folder).'
    )
    parsed_args, unknown = parser.parse_known_args(args=args)

    # Fixed base directory as requested
    base_dir = '/home/juli/coding/ROS-projekts/safety_demo/utils/data'
    full_path = os.path.normpath(os.path.join(base_dir, parsed_args.filename))

    rclpy.init(args=unknown)
    node = StateLogger(full_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
