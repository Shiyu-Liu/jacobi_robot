import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import numpy as np
import transforms3d as tf3d

class TFListenerNode(Node):
    """
    A TF Listener Node that extracts transformation from base link to end-effector frame, 
    used as a ground truth to compare with the desired EE pose.
    """
    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz
        self.T_6_EE_rviz = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]) # end-effector pose in frame 6 in rviz

    def timer_callback(self):
        try:
            # Lookup transform from 'base_link' to 'last_link'
            t: TransformStamped = self.tf_buffer.lookup_transform(
                'linear_axis_base_link',    # target frame
                'abb_arm_link_6',    # source frame
                rclpy.time.Time())

            T_b_6 = np.eye(4)
            T_b_6[0:3,0:3] = tf3d.quaternions.quat2mat([t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z])
            T_b_6[0:3,3] = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            T_b_EE = T_b_6 @ self.T_6_EE_rviz
            pos = T_b_EE[0:3,3]
            ori = tf3d.quaternions.mat2quat(T_b_EE[0:3,0:3])

            self.get_logger().info(f"Real EE Pose: Position = ({pos})")
            self.get_logger().info(f"              Orientation = ({ori})")

        except Exception as e:
            self.get_logger().warn(f"Transform not found: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TFListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()