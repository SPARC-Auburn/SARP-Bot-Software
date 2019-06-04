
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Cytron_SmartDriveDuo.h>
#include <ros.h>
#include <math.h>
#include <std_msgs/Int8.h>
#include <SoftwareSerial.h>

// Constants
#define IN1 4 // Arduino pin 4 is connected to MDDS60 pin IN1.
#define MOTOR_CONTROLLER_BAUDRATE  9600
#define ROS_BAUDRATE 57600
#define LEFT_DRIVE_POLARITY -1
#define RIGHT_DRIVE_POLARITY -1

// Variables
float leftDriveSpeed = 0;
float rightDriveSpeed = 0;

// Setup Motor Controller
Cytron_SmartDriveDuo smartDriveDuo30(SERIAL_SIMPLFIED, IN1, MOTOR_CONTROLLER_BAUDRATE);

// Setup ROS Communication
SoftwareSerial portROS(0, 1);
ros::NodeHandle nh;

// Update values from ROS
void leftDriveCallback(const std_msgs::Int8& leftDrive) 
{
  leftDriveSpeed = int(leftDrive.data);  
  smartDriveDuo30.control(leftDriveSpeed*LEFT_DRIVE_POLARITY,rightDriveSpeed*RIGHT_DRIVE_POLARITY); 
}

void rightDriveCallback(const std_msgs::Int8& rightDrive) 
{
  rightDriveSpeed = int(rightDrive.data);
  smartDriveDuo30.control(leftDriveSpeed*LEFT_DRIVE_POLARITY,rightDriveSpeed*RIGHT_DRIVE_POLARITY); 
}

ros::Subscriber<std_msgs::Int8> lsubscriber("lmotor_cmd", leftDriveCallback);
ros::Subscriber<std_msgs::Int8> rsubscriber("rmotor_cmd", rightDriveCallback);


// Setup serial and pin states
void setup()
{
  portROS.begin(ROS_BAUDRATE);
  nh.initNode();
  nh.subscribe(lsubscriber);
  nh.subscribe(rsubscriber);
  delay(2000);
}

void loop()
{
  nh.spinOnce();
  delay(1);
  
}
