sleep 1
roslaunch differential_drive diff-drive.launch &
roslaunch joystick.launch &
roslaunch arduino.launch &
roslaunch teensy.launch &
# roslaunch odometry_publisher odometry_publisher.launch &

