# marine_towing

## Overview

`marine_towing` is a ROS2 package for simulating and controlling marine towing systems. It provides a ROS2 node (`marine_model_node`) that computes the dynamics of a marine towing scenario using a cable and payload model. The node subscribes to state and force topics, computes the system's acceleration, and publishes the result.

## Features
- ROS2 node for marine towing simulation
- Uses Eigen for matrix computations
- Subscribes to state and force topics
- Publishes computed marine state

## Build Instructions

1. Source your ROS2 environment:
	```bash
	source /opt/ros/<your_distro>/setup.bash
	```
2. Install Eigen3 if not already installed:
	```bash
	sudo apt install libeigen3-dev
	```
3. Build the workspace:
	```bash
	cd ~/Documents/prosjekt/ros2_ws
	colcon build --packages-select marine_towing
	source install/setup.bash
	```

## Usage

1. Run the marine model node:
	```bash
	ros2 run marine_towing marine_model_node
	```
2. Publish test data to the required topics (in separate terminals):
	```bash
	ros2 topic pub /theta_topic std_msgs/msg/Float64 "{data: 0.5}"
	ros2 topic pub /theta_dot_topic std_msgs/msg/Float64 "{data: 0.1}"
	ros2 topic pub /F_0_topic std_msgs/msg/Float64MultiArray "{data: [1.0, 0.0]}"
	ros2 topic pub /F_topic std_msgs/msg/Float64MultiArray "{data: [0.0, 1.0]}"
	ros2 topic pub /F_u_topic std_msgs/msg/Float64MultiArray "{data: [0.5, 0.5]}"
	```
3. Echo the output:
	```bash
	ros2 topic echo /marine_state
	```

## Node Topics

- **Subscribed:**
  - `/theta_topic` (`std_msgs/msg/Float64`): Payload angle
  - `/theta_dot_topic` (`std_msgs/msg/Float64`): Payload angular velocity
  - `/F_0_topic` (`std_msgs/msg/Float64MultiArray`): External force on USV
  - `/F_topic` (`std_msgs/msg/Float64MultiArray`): Force on payload
  - `/F_u_topic` (`std_msgs/msg/Float64MultiArray`): Control force from USV thrusters
- **Published:**
  - `/marine_state` (`std_msgs/msg/Float64MultiArray`): Computed state (acceleration)

## License

See `package.xml` for license information.