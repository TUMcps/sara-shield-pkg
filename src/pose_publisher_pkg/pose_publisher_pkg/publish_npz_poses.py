import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
from scipy.spatial.transform import Rotation as R

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(PoseArray, '/target_poses', 10)
        
        try:
            # Load recorded 6D poses
            data_path = '/home/juli/coding/ROS-projekts/safety_demo/src/pose_publisher_pkg/data/pose_recording_dp.npz'
            data = np.load(data_path)
            target_poses = data['target_poses_6d']
            self.get_logger().info(f"Original target_poses shape: {target_poses.shape}")
            
            # Remove consecutive duplicates
            pose_changes = np.any(np.diff(target_poses, axis=0) != 0, axis=1)
            target_mask = np.concatenate([[True], pose_changes])
            target_poses = target_poses[target_mask]
            self.get_logger().info(f"Filtered target_poses shape: {target_poses.shape}")
            
            # Group poses into sequences of n
            n = 8
            num_sequences = len(target_poses) // n
            self.sequences = target_poses[:num_sequences * n].reshape(num_sequences, n, 6)
            self.index = 0
            
            if len(self.sequences) > 0:
                publish_interval = 0.2  # seconds (2 Hz)
                self.timer = self.create_timer(publish_interval, self.publish_next_sequence)
                self.get_logger().info(f"Prepared {num_sequences} pose sequences.")
            else:
                self.get_logger().warn("No valid pose sequences to publish.")
                
        except Exception as e:
            self.get_logger().error(f"Failed to load data: {e}")
            self.sequences = []
    
    def euler_to_quaternion(self, rx, ry, rz):
        """Convert euler angles to quaternion using scipy"""
        r = R.from_euler('xyz', [rx, ry, rz])
        quat = r.as_quat()  # Returns [x, y, z, w]
        return quat
    
    def publish_next_sequence(self):
        if self.index >= len(self.sequences):
            self.get_logger().info("Finished publishing all sequences.")
            self.timer.cancel()
            return
        
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "world"
        
        for pose in self.sequences[self.index]:
            x, y, z, rx, ry, rz = pose
            quat = self.euler_to_quaternion(rx, ry, rz)
            
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation.x = quat[0]  # x
            p.orientation.y = quat[1]  # y
            p.orientation.z = quat[2]  # z
            p.orientation.w = quat[3]  # w
            pose_array.poses.append(p)
        
        self.publisher_.publish(pose_array)
        self.get_logger().info(f"Published sequence {self.index + 1}/{len(self.sequences)}")
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()