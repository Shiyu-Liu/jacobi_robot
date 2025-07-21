import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_ik_solver = DeclareLaunchArgument(
        'use_ik_solver',
        default_value='true',
        description='whether to start the ik server node, otherwise use joint_state_publisher_gui'
    )

    use_ik_solver_arg = LaunchConfiguration('use_ik_solver')

    pkg_share = get_package_share_directory('jacobi_robot')

    robot_urdf = os.path.join(pkg_share, 'urdf', 'combined_robot.urdf')
    with open(robot_urdf, 'r') as file:
        robot_description = file.read()

    rviz_config = os.path.join(pkg_share, 'config', 'robot_config.rviz')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'gui': 'false'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    gazebo_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'jacobi_robot', '-topic', 'robot_description'],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=UnlessCondition(use_ik_solver_arg)
    )

    ik_solver_node = Node(
        package='jacobi_robot',
        executable='ik_solver_node',
        name='ik_solver_node',
        output='screen',
        condition=IfCondition(use_ik_solver_arg)
    )

    ee_pose_publisher = Node(
        package='jacobi_robot',
        executable='ee_pose_gui_node',
        name='ee_pose_gui_node',
        output='screen',
        condition=IfCondition(use_ik_solver_arg)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        gazebo_spawn_node,
        joint_state_publisher,
        rviz_node,
        ik_solver_node,
        ee_pose_publisher,
    ])