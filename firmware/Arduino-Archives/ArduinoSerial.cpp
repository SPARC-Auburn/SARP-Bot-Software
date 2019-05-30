
/*------------------------------------------------------------------------------
Filename:     ArduinoSerial.cpp
Project:      IEEE SoutheastCon Hardware Competition 2019
School:       Auburn University
Organization: Student Projects and Research Committee (SPARC)
Description:  Communicates with an Arduino from the Raspberry Pi 3 B+ over USB.
Controls 2 drive motors and 3 steppers.  Speed ranges are from -127 to 127.
------------------------------------------------------------------------------*/
#include "ArduinoSerial.h"
#include <stdexcept>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>		//for strerror()
#include <sstream>
#include <iostream>
#include <string>

// Constants
#define DEBUG_TEXT 0
const char serialPort::typicalPortName[] = "/dev/ttyUSB1";

// Namespaces
using namespace std;

// Variables
int leftDriveSpeed = 0;
int rightDriveSpeed = 0;
int gatePos = 0;
int flagPos = 0;
string LCDtext = "Connected!";
string buttonState = "0";
int clearButtonState = 0;
string mode = "-1";


serialPort::serialPort(const char* portName) {
  fileHandle = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
  if(fileHandle == -1) {
    throw std::runtime_error(string("Error opening port: ") + strerror(errno));
  }
  if(!isatty(fileHandle)) {
    close(fileHandle);
    throw std::runtime_error("Port is not a serial device.");
  }

  cfmakeraw(&config);     //Sets various parameters for non-canonical mode; disables parity

  cfsetospeed (&config, B38400);    //Baud rate
  cfsetispeed (&config, B38400);

  config.c_cflag     &=  ~CSTOPB;    //One stop bit

  config.c_cflag     |=  CREAD | CLOCAL;
  config.c_cflag     &=  ~CRTSCTS;           // no flow control

  config.c_cc[VMIN]   =  0;
  config.c_cc[VTIME]  =  0;

  if(tcsetattr(fileHandle, TCSANOW, &config) != 0) {
    close(fileHandle);
    throw std::runtime_error("Setting attributes failed.");
  }
  usleep(1000*1000*1); // wait 1 sec for arduino to reset
  tcflush(fileHandle, TCIFLUSH);    //clear input buffer
  usleep(1000*1000*1); // wait 1 sec for arduino to reset
}

void serialPort::write(string text) {
  ssize_t bytes_written = ::write(fileHandle, text.c_str(), text.length());
}

void serialPort::write(char data[], int length) {
  ssize_t bytes_written =  ::write(fileHandle, data, length);
}

string serialPort::read() {
  string input;
  int bytes = available();
  input.resize(bytes);
  ssize_t bytes_written = ::read(fileHandle, const_cast<char*>(input.data()) , bytes);    //The const cast is less than ideal
  return(input);
}

int serialPort::available() {
  int bytes;
  ioctl(fileHandle, FIONREAD, &bytes);
  return(bytes);
}

// Updates the motor speeds according to the state variables
string serialPort::updateArduino() {
  signed char char1 = (signed char)(leftDriveSpeed);
  signed char char2 = (signed char)(rightDriveSpeed);
  unsigned char char3 = (unsigned char)(gatePos);
  unsigned char char4 = (unsigned char)(flagPos);
  unsigned char char6 = (unsigned char)(clearButtonState);
  string newText = LCDtext;
  newText.resize(32 , ' ');
  unsigned char char5 = char1 + char2 + char3 + char4 + char6 + 1;
  char startChar[1] = {'<'};
  signed char motorData[2] = {char1,char2};
  unsigned char servoData[4] = {char3, char4, char6, char5};
  char endChar[1] = {'>'};
  if (DEBUG_TEXT){
    cout << "Sending to Arduino: " << startChar[0] << (int)char1 << "," << (int)char2 << "," << (int)char3  << "," << (int)char4 << "," << (int)char6 << ",";
    cout << (int)char5 << ","  << newText << "," << endChar[0] <<  endl;
   // cout << "Total bytes: " << (6+newText.length()) << endl;
  }
  ssize_t bytes_written = ::write(fileHandle, startChar, 1); 
  bytes_written = ::write(fileHandle, motorData, 2); //Send messages
  bytes_written = ::write(fileHandle, servoData, 4); //Appended checksum
  bytes_written = ::write(fileHandle, newText.c_str(), newText.length());
  bytes_written = ::write(fileHandle, endChar, 1);
 // if (DEBUG_TEXT){ //Output text characters
 // 	char qqqq[32];
 // 	memcpy(qqqq,newText.c_str(),32);
 // 	for(int i = 0; i<32; i++)cout << (int)qqqq[i] << " ";
 // 	cout << endl;
 // }
  string received = read();
  string delim = ","; //Pick out 6th value to find buttonState
  auto start = 0U;
  auto end = received.find(delim);
  int value = 0;
  if (DEBUG_TEXT){
	cout << "Recieved from Arduino: ";
  }
  while (end != string::npos){
	value++;
	if(value==8){
		buttonState = received.substr(start, end - start);
		if (DEBUG_TEXT){
			cout << "ButtonState: " << buttonState << ",";
		}
	}
	else if(value==9){
		mode = received.substr(start, end - start);
		if (DEBUG_TEXT){
			cout << "Mode: " << mode;
		}
		break;
	}
	else if (DEBUG_TEXT){
		 cout << received.substr(start, end - start) << ",";
	}
	start = end + delim.length();
	end = received.find(delim, start);
  }
  if(DEBUG_TEXT)
    cout << endl;
  return buttonState;
}

void serialPort::turnLeft(int speed){
  if(speed < 0 || speed > 127)
		throw out_of_range("Motor speed must be between 0 and 127.");
  leftDriveSpeed = speed;
  rightDriveSpeed = -speed;
}

void serialPort::turnRight(int speed){
  if(speed < 0 || speed > 127)
		throw out_of_range("Motor speed must be between 0 and 127.");
  leftDriveSpeed = -speed;
  rightDriveSpeed = speed;
}

void serialPort::goForward(int speed){
  if(speed < 0 || speed > 127)
		throw out_of_range("Motor speed must be between 0 and 127.");
  leftDriveSpeed = -speed;
  rightDriveSpeed = -speed;
}

void serialPort::goBackward(int speed){
  if(speed < 0 || speed > 127)
  		throw out_of_range("Motor speed must be between 0 and 127.");
  leftDriveSpeed = speed;
  rightDriveSpeed = speed;
}
void serialPort::drive(int left, int right){
  leftDriveSpeed = left;
  rightDriveSpeed = right;
}
void serialPort::stopMotors(){
  leftDriveSpeed = 0;
  rightDriveSpeed = 0;
}

void serialPort::moveGate(int pos){
  if(pos < 0 || pos > 180)
		throw out_of_range("Servo position must be between 0 and 180.");
  gatePos = pos;
}

void serialPort::moveFlag(int pos){
  if(pos < 0 || pos > 180)
		throw out_of_range("Servo position must be between 0 and 180.");
  flagPos = pos;
}

void serialPort::updateLCD(string text){
  LCDtext = text;
  if(DEBUG_TEXT){
 	 cout << "New LCD text: " << text << endl;
  }
}

int serialPort::getMode(){
	return stoi(mode);
}

int serialPort::getButtonState(){
	int currentState = 0;
	currentState = stoi(buttonState);
	if (buttonState == "1"){
		clearButtonState = 1;
		buttonState = "0";
	}
	else{
		clearButtonState = 0;
	}
	return currentState;
}


serialPort::~serialPort() {
  cout << "Disconnecting from Arduino..." << endl;
  updateLCD("Disconnected..");
  updateArduino();
  close(fileHandle);
}
