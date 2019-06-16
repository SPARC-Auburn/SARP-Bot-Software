sleep 1
roslaunch differential_drive diff-drive.launch &
roslaunch arduino.launch &
roslaunch teensy.launch &
x-terminal-emulator -e rosrun teleop_twist_keyboard teleop_twist_keyboard.py
# roslaunch odometry_publisher odometry_publisher.launch &

