
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Cytron_SmartDriveDuo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

// Constants
#define IN1 4 // Arduino pin 4 is connected to MDDS60 pin IN1.
#define MOTOR_CONTROLLER_BAUDRATE  9600
#define leftDrivePolarity -1
#define rightDrivePolarity -1

// Variables
signed char leftDriveSpeed;
signed char rightDriveSpeed;

ros::NodeHandle  nh;

Cytron_SmartDriveDuo smartDriveDuo30(SERIAL_SIMPLFIED, IN1, MOTOR_CONTROLLER_BAUDRATE);


void servo_cb( const std_msgs::UInt16& cmd_msg){
  smartDriveDuo30.control(0,int(cmd_msg.data)); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

// Setup serial and pin states
void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
