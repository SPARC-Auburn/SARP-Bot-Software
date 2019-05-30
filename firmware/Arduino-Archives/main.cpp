#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include <thread>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include "opencv_node/vision_msg.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "opencv_node/object.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

//#include "sensor_msgs/Imu.h"
#include "Arduino-Serial/ArduinoSerial.h"
//Color Indices = red(0), yellow(1), blue(2), green(3)
using namespace std;

//serialPort arduino("/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0");


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std_msgs::String msg;
int rightSpeed=0,leftSpeed=0;
double closestBlockX = 0.0;//add this to y for map placement
double closestBlockY = 0.0;//add this to x for map placement
int numberBlocks = 0; //number of blocks seen
double desiredColor = 0.0;
string colorSelect = "0"; //recieved startColor
int colorChoose = 0;
int goalMet = 0;
int octetNum = 0;
double dummyRobotX = 0.0;
double dummyRobotY = 0.0;
int startMatch = 0;
int loopNum = 0;
double initialPose[2] = {0.0,0.0};
int moveBaseTest = 0;

void colorSelected(const std_msgs::Float32ConstPtr &msg){
  colorChoose = int(msg->data);
}
void matchStarted(const std_msgs::Float32ConstPtr &msg){
	startMatch = int(msg->data);
}

double objDistance(const opencv_node::object& obj) {
	return sqrt(pow(obj.x_position, 2) + pow(obj.y_position, 2));
}

void visionCallback(const opencv_node::vision_msg::ConstPtr &msg)
{
	ROS_INFO("Main>>>Number of Objects: %d", msg->objects.size());
  numberBlocks = msg->objects.size();
	int desiredColor = 0;
	double minDistance = 0.0;
	int currentMin = -1;
	for (int i = 0; i < msg->objects.size(); ++i)
	{
		const opencv_node::object &prop = msg->objects[i];
		ROS_INFO_STREAM("Position: " << prop.x_position << "," << prop.y_position << " Color:" << prop.color_index << " Object Type:" << prop.object_type);
		if(prop.color_index == desiredColor && (currentMin == -1 || objDistance(prop) < minDistance)) {
			currentMin = i;
			minDistance = objDistance(prop);
		}
	}
	//auto closest = min_element(msg->objects.begin(),msg->objects.end(),[](const opencv_node::object &first,const opencv_node::object &second){
		//return objDistance(first) < objDistance(second);
	//});
	if(currentMin != -1) {
    closestBlockX = msg->objects[currentMin].x_position;
    closestBlockY = msg->objects[currentMin].y_position;
    desiredColor = msg->objects[currentMin].color_index;//not used yet
		ROS_INFO_STREAM("Selected Object >>> Position: " << msg->objects[currentMin].x_position << "," << msg->objects[currentMin].y_position << " Color:" << msg->objects[currentMin].color_index << " Object Type:" << msg->objects[currentMin].object_type);
	}
	
	  
}

void moveFwdOneMeter(){
	//MOVE BASE CODE//
	int counter = 1;
        MoveBaseClient ac("/move_base", true); //Tell the client we want to spin a thread by default
         while(!ac.waitForServer(ros::Duration(5.0))){
                ROS_INFO("Waiting for the move_base action server to come up");
        }

        move_base_msgs::MoveBaseGoal moveFwd;
	
	moveFwd.target_pose.header.frame_id = "base_footprint";
        moveFwd.target_pose.header.stamp = ros::Time::now();
	
	//if(counter){
        	moveFwd.target_pose.pose.position.x = 0.5; //move 1 meter forward
		moveFwd.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0); 
		counter = 0;
	//}
	//else{
	/*	counter++;
		moveFwd.target_pose.pose.position.x = 0.5;
		moveFwd.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-90.0);
	}*/
        ROS_INFO("Sending goal");
	ac.sendGoal(moveFwd);

	//ac.waitForResult();

	//if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	//	ROS_INFO("WOOP WOOP YOU DID IT");
	//else
	//	ROS_INFO("You screwed up boi");
        /////////////////

}

//Note: frame is typicall "map" or "base_footprint"
bool moveToGoal(double xGoal, double yGoal){
   //define a client for to send goal requests to the move_base server through a SimpleActionClient
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("/move_base", true);

   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "map";
   goal.target_pose.header.stamp = ros::Time::now();

   /* moving towards the goal*/

   goal.target_pose.pose.position.x = - xGoal;
   goal.target_pose.pose.position.y = - yGoal;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Sending goal location ...");
   ac.sendGoal(goal);

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("You have reached the destination");
      goalMet = 1;
      return true;
   }
   else{
      ROS_INFO("The robot failed to reach the destination");
      goalMet = 0;
      return false;
   }

}

// void arduinoCallback(const std_msgs::String& msg) {
// 	ROS_INFO_STREAM(msg << '\n');
// }

/*void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  float gyro_x = msg->angular_velocity.x;
  float gyro_y = msg->angular_velocity.y;
  float gyro_z = msg->angular_velocity.z;
  float orientation_x = msg->orientation.x;
  float orientation_y = msg->orientation.y;
  float orientation_z = msg->orientation.z;
  float orientation_w = msg->orientation.w;
  //ROS_INFO("Main>>>Angular Velocity: x(%f),y(%f),z(%f)", gyro_x,gyro_y,gyro_z);
  ROS_INFO("Main>>>Orientation: x(%f),y(%f),z(%f)", orientation_x, orientation_y, orientation_z);
}*/


int main(int argc, char **argv)
{

  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("vision_info", 1000, visionCallback);



 // initPose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1,true);

  cout << "\033[1;34m-------------------------------------------------------------------\033[0m" << endl;
  cout << "\033[1;34m   .:: ::    .:::::::          .:         .:::::::          .::    \033[0m" << endl;
  cout << "\033[1;34m .::    .::  .::    .::       .: ::       .::    .::     .::   .:: \033[0m" << endl;
  cout << "\033[1;34m  .::        .::    .::      .:  .::      .::    .::    .::        \033[0m" << endl;
  cout << "\033[1;34m    .::      .:::::::       .::   .::     .: .::        .::        \033[0m" << endl;
  cout << "\033[1;34m       .::   .::           .:::::: .::    .::  .::      .::        \033[0m" << endl;
  cout << "\033[1;34m .::    .::  .::          .::       .::   .::    .::     .::   .:: \033[0m" << endl;
  cout << "\033[1;34m   .:: ::    .::         .::         .::  .::      .::     .::::   \033[0m" << endl;
  cout << "\033[1;34m-------------------------------------------------------------------\033[0m" << endl;
  cout << "\033[1;34m|        Student Projects and Research Committee IEEE 2019        |\033[0m" << endl;
  cout << "\033[1;34m ------------------------------------------------------------------\033[0m" << endl;
  sleep(1);
  //ros::Subscriber sub2 = n.subscribe("sensor_msgs/Imu", 1000, imuCallback);

  ros::Publisher colorSelectPub = n.advertise<std_msgs::Int32>("colorSelect",1);//this publishes data to vision shit
  ros::Publisher gate_cmd = n.advertise<std_msgs::Int32>("gate_cmd",1);
  ros::Publisher flag_cmd = n.advertise<std_msgs::Int32>("flag_cmd",1);
  ros::Publisher color_Want = n.advertise<std_msgs::Float32>("color_want",1);

  ros::Subscriber startColorSub = n.subscribe<std_msgs::Float32>("start_color", 1, colorSelected);
  ros::Subscriber startMatchSub = n.subscribe<std_msgs::Float32>("start_match", 1, matchStarted);
	ros::Rate loop_rate(40);	//1 Hz
  

	//geometry_msgs::PoseWithCovarianceStamped ip;
	//ip.header.frame_id = "map";
	ros::Time current_time = ros::Time::now();
	/*ip.header.stamp = current_time;
	ip.pose.pose.position.x = 0.1143;
	ip.pose.pose.position.y = 0.1143;
	ip.pose.pose.orientation.z = 0;
	ip.pose.covariance[0] = 1e-3;
	ip.pose.covariance[7] = 1e-3;
	ip.pose.covariance[35] = 1e-3;
	initPose.publish(ip);
	*/
	int count = 0;
	bool done = 0;
  double myGoalX[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  double myGoalY[8] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};//set these according to the new map locations

	while(ros::ok()) {
      	
		msg.data = std::string("Hello ");
		msg.data += std::to_string(count);
    //this sets the selected 
    

    //this tests the octet and debris goal setting
    if(startMatch == 1){ //only runs on second button press
      
      if(octetNum == 0){//first start location setting
        moveToGoal(myGoalX[octetNum],myGoalY[octetNum]);//go to initial octet
        if(goalMet){octetNum++;}
      }

      //check to see if any blocks in view, if so go to them.
      if(numberBlocks > 0 && octetNum != 0){ 
        
        
        gate_cmd.publish(75);//open gate
        moveToGoal(dummyRobotX+closestBlockY+0.11,dummyRobotY+closestBlockX);//location reference to map, .11 is roughly half the width of the robot 
        //to compensate for camera offset,,this waits until the robot has met its goal
        //also this moves to collect all blocks
        if(goalMet){octetNum++;}

      }
      else if(numberBlocks == 0 && abs(dummyRobotX-myGoalX[octetNum+1])>0.2 && abs(dummyRobotY-myGoalY[octetNum+1])<0.2){ //checks to see if goal is too close to current position, 20 is arbitrary
        gate_cmd.publish(20);//close Gate
        octetNum++;
      }
      else{
        moveToGoal(myGoalX[octetNum],myGoalY[octetNum]);
      }

      if(octetNum == 8){octetNum = 0;loopNum++;color_Want.publish(colorChoose++);}

      if(loopNum == 3){
        moveToGoal(initialPose[0],initialPose[1]);//go back to start position
          if(goalMet){octetNum++;loopNum = 0; flag_cmd.publish(40);}
      }
    }
		if(moveBaseTest == 0){
			moveBaseTest++;
			moveToGoal(-0.5,0);
			//moveFwdOneMeter();
		}
		
		ros::spinOnce();	
		loop_rate.sleep();
		++count;
	}

  return 0;
}

