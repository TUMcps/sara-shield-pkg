#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class HumanMotionTracker(Node):
    def __init__(self):
        super().__init__('human_motion_tracker')
        self.pub = self.create_publisher(Float32MultiArray, 'human_measurements', 10)
        timer_period = 0.001  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_measurements)
        self.get_logger().info('HumanMotionTracker initialized, publishing dummy measurements at 10Hz')

        # Default dummy measurement points (23 points) for mujoco_mocap.yaml
        # self.points = [
        #     (1.0786, -0.0677, 0.3353),
        #     (1.0761,  0.0695, 0.3361),
        #     (1.0985,  0.0043, 0.5356),
        #     (1.0836, -0.1048,-0.0396),
        #     (1.0882,  0.1091,-0.0462),
        #     (1.0974, -0.0012, 0.6708),
        #     (1.1111, -0.0851,-0.4388),
        #     (1.1077,  0.0932,-0.4463),
        #     (1.0720, -0.0026, 0.7237),
        #     (0.9878, -0.1138,-0.4839),
        #     (0.9800,  0.1200,-0.4806),
        #     (1.1149,  0.0002, 0.9376),
        #     (1.1060, -0.0815, 0.8455),
        #     (1.1106,  0.0791, 0.8426),
        #     (1.0635, -0.0050, 1.0026),
        #     (1.1149, -0.1724, 0.8760),
        #     (1.1197,  0.1752, 0.8751),
        #     (1.1424, -0.4320, 0.8631),
        #     (1.1411,  0.4289, 0.8617),
        #     (1.1435, -0.6813, 0.8721),
        #     (1.1467,  0.6842, 0.8695),
        #     (1.1585, -0.7653, 0.8639),
        #     (1.1570,  0.7688, 0.8633)
        # ]
        # for human_reach_TUM_lab.yaml
        self.points = [
            (0.0, -10.0, 0.0),
            (0.0, -10.0, 0.0),
            (0.033, -0.614, -0.1),
            (0.0, -10.0, 0.0),
            (0.0, -10.0, 0.0),
            (0.0, -10.0, 0.0),
            (0.0, -10.0, 0.0),
            (0.0, -10.0, 0.0),
            (0.0, -10.0, 0.0),
        ]

    def publish_measurements(self):
        msg = Float32MultiArray()
        # Flatten to [x1,y1,z1, x2,y2,z2, ...]
        msg.data = [v for point in self.points for v in point]
        self.pub.publish(msg)
        self.get_logger().debug(f'Published {len(self.points)} points')


def main(args=None):
    rclpy.init(args=args)
    node = HumanMotionTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
