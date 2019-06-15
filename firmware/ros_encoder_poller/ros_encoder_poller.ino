#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <ros.h>
#include <math.h>
#include <std_msgs/Int16.h>

// Setup Encoder Pins
Encoder myEnc(5, 6);
Encoder myEnc2(11, 12);

// Variables
std_msgs::Int16 l;
std_msgs::Int16 r;

// Setup ROS Communication
ros::NodeHandle nh;
ros::Publisher pub_left("lwheel", &l);
ros::Publisher pub_right("rwheel", &r);

// Setup serial and pin states
void setup()
{
  nh.initNode();
  nh.advertise(pub_left);
  nh.advertise(pub_right);
  delay(2000);
}

void loop()
{
  l.data = (int16_t)myEnc.read();
  r.data = (int16_t)myEnc2.read();
  pub_left.publish(&l);
  pub_right.publish(&r);
  nh.spinOnce();
  delay(10);
}
