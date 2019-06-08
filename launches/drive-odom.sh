sleep 1
roslaunch test.launch &
roslaunch teleop.launch &
rosrun rosserial_python serial_node.py /dev/ttyUSB0 &
roslaunch odometry_publisher odometry_publisher.launch &
python ../catkin_ws/odom.py &
echo "Use L shoulder button on controller to enable driving.  Use R shoulder button for turbo mode."
