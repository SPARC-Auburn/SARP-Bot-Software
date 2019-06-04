##Arduino Setup
Arduino Used: https://www.amazon.com/Longruner-ATmega328P-Controller-Module-Arduino/dp/B01MSYWE6B/ref=sr_1_2_sspa?keywords=arduino+microphone&qid=1559157394&s=gateway&sr=8-2-spell-spons&psc=1
Download IDE: https://www.arduino.cc/en/Main/Software
Board: Arduino Nano
Port: ttyUSBx
Processor: ATmega328P(old bootloader)
Programmer: AVRISP mkll

Copy contents of library folder to `~/Arduino/libraries`

##Setup ROS Serial
```
sudo apt-get install ros-melodic-rosserial-arduino
sudo apt-get install ros-melodic-rosserial
sudo apt-get install ros-melodic-teleop-twist-keyboard
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/rosserial.git
cd ..
catkin_make
cd ~/Arduino
rosrun rosserial_arduino make_libraries.py .
```

##Running Arduino with ROS
Change port name as needed
```rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=57600```