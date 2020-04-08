7#!/usr/bin/zsh

echo "Publishing constantly to ROS - twist"

# reads custom values
echo "input linear velocity:"
read vel_linear
echo "input angular velocity:"
read vel_angular

# publishing to ROS
echo "Publishing to ROS..."


rostopic pub -r 1 /cmd_vel geometry_msgs/Twist "{linear: {x: $vel_linear, y: 0, z: 0}, angular: {x: 0, y: 0, z: $vel_angular}}"

