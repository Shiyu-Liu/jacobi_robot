import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
qos_profile_sensor_data.depth = 1

import tkinter as tk
from tkinter import ttk
from functools import partial
import transforms3d as tf3d
import math


class EEPosePublisher(Node):
    """
    An End-Effector Pose Publisher that has a simple interface to publish desired end-effector pose in ROS2.

    The node has a tkinter interface to set up the position and orientation (roll-pitch-yaw Euler angles) of the desired end-effector pose,
    and publishes the pose to the topic /desired_ee_pose.
    """
    def __init__(self):
        super().__init__('ee_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/desired_ee_pose', qos_profile=qos_profile_sensor_data)
        self.get_logger().info('EE Pose Publisher started.')

    def publish_pose(self, fields):
        try:
            # read values from the GUI fields
            pos = [float(fields[i].get()) for i in range(3)]
            eul = [float(fields[i].get()) for i in range(3,6)]

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'linear_axis_base_link' # set customized base_link frame

            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            msg.pose.position.z = pos[2]

            ori = tf3d.euler.euler2quat(eul[0]*math.pi/180., eul[1]*math.pi/180., eul[2]*math.pi/180., 'sxyz')
            msg.pose.orientation.w = ori[0]
            msg.pose.orientation.x = ori[1]
            msg.pose.orientation.y = ori[2]
            msg.pose.orientation.z = ori[3]

            self.publisher_.publish(msg)
            self.get_logger().info(f'Desired EE pose: {pos}, {ori}')
        except ValueError:
            self.get_logger().error('Invalid input! Make sure all fields are numbers.')


    def start_gui(self):
        root = tk.Tk()
        root.title("EE Pose Input")

        labels = ['x (m)', 'y (m)', 'z (m)', 'roll (°)', 'pitch (°)', 'yaw (°)']
        fields = []

        for i, label in enumerate(labels):
            ttk.Label(root, text=label).grid(row=i, column=0, padx=5, pady=5)
            entry = ttk.Entry(root)
            entry.grid(row=i, column=1, padx=5, pady=5)
            fields.append(entry)

        # default values (when joint positions are all zero)
        default_values = [0.6, 2.5425, 2.81, -90.0, 0.0, 0.0]
        for entry, val in zip(fields, default_values):
            entry.insert(0, str(val))

        send_button = ttk.Button(root, text='Send Pose', command=partial(self.publish_pose, fields))
        send_button.grid(row=7, column=0, columnspan=2, pady=10)

        root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = EEPosePublisher()

    try:
        node.start_gui()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()