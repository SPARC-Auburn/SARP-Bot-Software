#include <Cytron_SmartDriveDuo.h>
//#include <std_msgs/String.h>
#include <Wire.h> 

// Constants
#define IN1 4 // Arduino pin 4 is connected to MDDS60 pin IN1.
#define MOTOR_CONTROLLER_BAUDRATE  9600
#define leftDrivePolarity -1
#define rightDrivePolarity -1

// Variables
signed char leftDriveSpeed;
signed char rightDriveSpeed;
Cytron_SmartDriveDuo smartDriveDuo30(SERIAL_SIMPLFIED, IN1, MOTOR_CONTROLLER_BAUDRATE);

// Setup serial and pin states
void setup()
{
  delay(1000);
  smartDriveDuo30.control(-20,-20); // Drive Forward for 1 second
  delay(1000);
  smartDriveDuo30.control(0,0);
  delay(1000);
  smartDriveDuo30.control(20,20); // Drive Backward for 1 second
  delay(1000);
  smartDriveDuo30.control(0,0);
}

void loop()
{
  
}
