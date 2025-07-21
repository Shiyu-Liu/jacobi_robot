# Jacobi_Robot
This repository implements an analytic inverse kinematics (IK) solver for a robot composed of a 6-DoF ABB arm mounted on a linear axis.
The joint configuration (values for one prismatic joint of the linear axis, as well as six revolute joints of ABB arm) is solved from a
given End-Effector (EE) pose.

## Prerequisites
Before you begin, ensure you have the following installed:

- ROS2 (humble distro) (https://docs.ros.org/en/humble/Installation.html) (Note: the code is tested in ROS2 humble under Ubuntu22.04, higher version like ROS2 jazzy with Ubuntu24.04 should be okay as well)

- Gazebo Classic (Gazebo 11 or higher) (https://classic.gazebosim.org/) (Note: the next step involves moving towards new Gazebo Ignition)

- colcon build tool (https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

## Code Structure
The structure of the repository is as follows:

- **config**: contains configuration files such as custom RViz configuration

- **doc**: includes inverse_kinematics_solution.pdf as a detailed analytic IK solution

- **jacobi_robot**: contains all python scripts (ROS-free or ROS-dependent)

- **launch**: ros launch files

- **test**: pytest scripts (currently empty files without any tests performed)

- **urdf**: all urdf files for robot modeling and simulation in gazebo


## Nodes
There are several nodes implemented in this repository, which are listed below in order of importance:

- **ik_solver_node**: wraps the inverse kinematics solver (implemented inside the ROS-free *ik_solver.py* script) in ROS enviroment

- **ee_pose_gui_node**: starts a simple panel interface allowing for defining the desired end-effector pose

- **tf_listener_node**: retrieves the actual end-effector pose from tf feedback

## Installation & Execution
Install the repository in a ros2 workspace as:
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/Shiyu-Liu/jacobi_robot.git
cd ~/ros_ws && colcon build --packages-select jacobi_robot
```

Execute by launching the file:
```bash
ros2 launch jacobi_robot sim_robot.launch.py use_ik_solver:=true
```
Set the parameter *use_ik_solver* to *True* to enable IK solver (enabled by default), otherwise another control panel is open allowing manual adjustment of joint positions by sliders.

Run a tf_listener to validate the found joint solutions, comparing the desired and actual EE pose (optional):
```bash
ros2 run jacobi_robot tf_listener_node
```

## Next steps
The repository at the current stage only validates the IK solution, and outputs the joint positions to allow the EE to reach its desired pose. The transition from the previous joint configuration to the new one is however instantaneous. The following steps will be:

- Implement JointTrajectoryController and control the joints using *gazebo_ros2_control* plugin

- Upgrade to new Gazebo (gazebo ignition) for longer support (Gazebo classic has reached end-of-life in Jan. 2025)

- Work on more realistic control (developing feedback control law, controlling by velocity with Jacobian, generating trajectory for joint configuration transitions as potential directions)