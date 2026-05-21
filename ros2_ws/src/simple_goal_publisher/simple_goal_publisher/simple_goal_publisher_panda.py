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
            [1.7, 1.40, 0.8557245716351515, -0.67643376135128, -0.15718006952603658, 3.358743019147018, 0.7184185826257936],
            [-0.47056755812951995, 0.9410700825585258, 1.1853199879239116, -0.6521574217310434, -0.3002404732369599, 2.8922996513627623, 0.9543601782592442],
            [-0.75, 1.2242576980256197, 0.5952448330664808, -0.7961084507222761, -0.3055968793597486, 2.6919983236856706, 0.8166998131787222],
            [-0.47056755812951995, 0.9410700825585258, 1.1853199879239116, -0.6521574217310434, -0.3002404732369599, 2.8922996513627623, 0.9543601782592442],
        ]
        self.tolerance = 0.02   # radians tolerance
        self.current_idx = 0

        # Publisher to safety shield
        self.goal_pub = self.create_publisher(JointState, 'goal_joint_states', 10)
        # Subscriber to get shield’s desired output
        self.desired_sub = self.create_subscription(JointState,
                                                    'desired_joint_states',
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
