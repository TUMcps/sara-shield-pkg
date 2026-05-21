#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class GoalSequencer(Node):
    def __init__(self):
        super().__init__('goal_sequencer')
        # hard‐coded list of goals (6 joints each)
        self.goals = [
            [1.0, 0.0, 0.0, 0.3, 0.5, 1.0],
            [1.2, 0.0, 0.0, 0.5, 0.7, 1.0],
            [1.5, 0.0, 0.0, 0.7, 0.9, 1.0],
            [1.0, 0.0, 0.0, 0.3, 0.5, 1.0],
            [1.0, 1.4, 0.0, -0.6, 0.3, 0.0],
            [0.0, 0.5, -0.9, 0.0, 0.0, 0.0],
            [0.0, 1.4, 0.0, 0.0, 0.5, 0.0],
            [-1.4, 1.4, 0.0, 0.5, 0.0, 0.5],
            [-2.0, 0.0, 0.0, 1.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        self.tolerance = 0.02   # radians tolerance
        self.current_idx = 0

        # Publisher to safety shield
        self.goal_pub = self.create_publisher(JointState, 'goal_joint_states', 10)
        # Subscriber to get shield’s desired output
        self.desired_sub = self.create_subscription(JointState,
                                                    'current_joint_states',
                                                    self.desired_cb, 10)

        # Immediately send the first goal
        self.publish_goal()

    def publish_goal(self):
        goal = self.goals[self.current_idx]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint{i+1}' for i in range(len(goal))]
        msg.position = goal
        self.goal_pub.publish(msg)
        self.get_logger().info(f'→ Published goal #{self.current_idx}: {["{:.2f}".format(x) for x in goal]}')

    def desired_cb(self, msg: JointState):
        # Check only if lengths match
        if len(msg.position) != len(self.goals[self.current_idx]):
            return
        # Compute max absolute error
        errs = [abs(p - g)
                for p, g in zip(msg.position, self.goals[self.current_idx])]
        # check if velocity is roughly zero
        if any(abs(v) > 0.001 for v in msg.velocity):
            return
        
        if max(errs) < self.tolerance:
            self.get_logger().info(f'✔ Goal #{self.current_idx} reached (err={max(errs):.3f})')
            # advance
            self.current_idx = (self.current_idx + 1) % len(self.goals)
            self.publish_goal()

def main(args=None):
    rclpy.init(args=args)
    node = GoalSequencer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
