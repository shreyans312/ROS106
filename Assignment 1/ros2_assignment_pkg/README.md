This package implements two nodes in ROS2 jazzy jalisco:
pose_filter_node: Filters noisy robot position by averaging the last 10 datas.
transform_service_node: Transforms the robot's position from the world frame to the Professor's frame.


I have assumed the professor's fixed position is assumed to be x = 4.0, y = 3.0 and theta = pi/4 as the assignment allowed any value of choice.

For verification of the package I have also created test client and test publisher nodes as well.

Link to the github repository for the package: https://github.com/shreyans312/ROS106/

Commands to run the package:
To build the package: colcon build --package-select ros2_assignment_pkg

Use 4 different terminal windows to run all the nodes and write the command: source install/setup.bash
To run the node write: ros2 run ros2_assignment_pkg NODE_NAME

![image](https://github.com/user-attachments/assets/68632a25-24fd-4d85-aee0-b1ac44ac27d5)
