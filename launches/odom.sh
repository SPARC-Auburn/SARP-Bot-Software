#rosrun tf2_ros static_transform_publisher 0.06 0 0 1.570796327 0 0 base_footprint base_imu_link &
#rosrun tf2_ros static_transform_publisher -0.06 0 0 0 0 0 base_footprint odom_footprint  &
roslaunch odometry_publisher odometry_publisher.launch &
python ../catkin_ws/odom.py &
#roslaunch ./razor-pub.launch &
#roslaunch ./ekf_template.launch &
