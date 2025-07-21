import numpy as np
import rclpy
from rclpy.node import Node
from jacobi_robot.ik_solver import InverseKinematicsSolver
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
qos_profile_sensor_data.depth = 1

class IKSolverNode(Node):
    """
    An IK Solver Node is a wrapper of Inverse Kinematics Solver in ROS2.

    The node receives desired end-effector pose from the topic /desired_ee_pose, calls the IK solver to find the solution of joint positions,
    and publishes the ouput to the topic /joint_states.
    """
    def __init__(self):
        super().__init__('ik_solver_node')

        # initialize IK solver
        self.ik_solver = InverseKinematicsSolver()

        # initialize joint names in accordance with Gazebo simulator
        self.joint_names = ['linear_axis_joint_1']
        self.joint_names.extend(f'abb_arm_joint_{i+1}' for i in range(6))

        # initialize joint configuration
        self.q = np.zeros(7)

        # subscriber to desired end-effector pose
        self.create_subscription(
            PoseStamped,
            '/desired_ee_pose',
            self.ee_pose_callback,
            qos_profile=qos_profile_sensor_data
        )

        # publisher for joint positions
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile=qos_profile_sensor_data)

        # create a timer for publishing joint states periodically
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('IK Solver Node has started.')

    def ee_pose_callback(self, msg: PoseStamped):
        # get desired pose
        position = msg.pose.position
        orientation = msg.pose.orientation

        # log received pose
        self.get_logger().info(f"Received EE Pose: Position = ({position.x:.4f}, {position.y:.4f}, {position.z:.4f})")
        self.get_logger().info(f"                  Orientation = ({orientation.w:.4f}, {orientation.x:.4f}, {orientation.y:.4f}, {orientation.z:.2f})")

        # convert pose to numpy arrays
        desired_pose = {
            'position': np.array([position.x, position.y, position.z]),
            'orientation': np.array([orientation.w, orientation.x, orientation.y, orientation.z])
        }

        # solve IK
        joint_positions, err_msg = self.solve_ik(desired_pose)

        # publish joint positions (ToDo: publish to JointTrajectoryController instead)
        if joint_positions is not None:
            self.q = np.array(joint_positions)
            self.publish_joint_states(joint_positions)
        else:
            self.get_logger().warn(f"{err_msg}")

    def timer_callback(self):
        self.publish_joint_states(self.q.tolist())

    def solve_ik(self, pose: dict):
        # extract desired position and orientation
        desired_position = pose['position']
        desired_orientation = pose['orientation']
        # solve inverse kinematics
        joint_positions, err_msg = self.ik_solver.solve(self.q, desired_position, desired_orientation)
        return joint_positions, err_msg

    def publish_joint_states(self, q: list):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = q
        msg.name = self.joint_names
        self.joint_pub.publish(msg)
        self.get_logger().info(f"Published joint positions: {q}")


def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
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