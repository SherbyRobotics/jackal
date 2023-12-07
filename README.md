# Jackal ROS Control Script

## Overview
This Python script provides a basic obstacle avoidance and safety control mechanism for a Jackal robot in a ROS (Robot Operating System) environment. The script subscribes to laser scan data, monitors the surroundings, and adjusts the robot's velocity commands to avoid collisions.

## Requirements
- ROS (Robot Operating System)
- Python 2.7 or later
- NumPy
- Jackal simulation environment

## Installation
1. Ensure that ROS is installed on your system.
2. Clone this repository into your ROS workspace.
   ~~~bash
   git clone https://github.com/your-username/your-repository.git
   ~~~
3. Build your ROS workspace.
   ~~~bash
   cd your-ros-workspace
   catkin_make
   ~~~

## Usage
1. Run the control script:
   ~~~bash
   roslaunch basic_pkg start_basic.launch 
   ~~~
2. Launch the Gazebo simulator for Jackal with the following command:
   ~~~bash
   roslaunch jackal_gazebo jackal_world.launch config:=front_laser
   ~~~



## Node Details
- **Node Name:** control_test
- **Subscribed Topics:**
  - `/scan`: Laser scan data from the robot's sensors.
  - `/front/scan`: Laser scan data specifically for the front portion of the robot.
  - `/twist_marker_server/cmd_vel`: Velocity commands from the twist marker server.
  - `/bluetooth_teleop/cmd_vel`: Velocity commands from Bluetooth teleoperation.
  - `/joy_teleop/cmd_vel`: Velocity commands from joystick teleoperation.
  - `/jackal_velocity_controller/odom`: Odometry data for monitoring the Jackal's speed.

## Parameters
- `front_scan_point`: Number of points used for the front laser scanner.

## Behavior
The script continuously monitors the laser scan data to detect obstacles in front of the Jackal. If an obstacle is too close (`safe_distance`), it triggers a safety mode, reducing or stopping the robot's linear velocity. The control script can be adjusted for specific applications by modifying the parameters and tuning the reaction time, decceleration, and other control parameters.

Feel free to customize and extend this script based on your specific requirements.
