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

// Constants
#define DEBUG_TEXT 0
const char serialPort::typicalPortName[] = "/dev/ttyUSB0";

// Namespaces
using namespace std;

// Variables
int leftDriveSpeed = 0;
int rightDriveSpeed = 0;
int leftGateSpeed = 0;
int rightGateSpeed = 0;
int spinnerSpeed = 0;

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

  cfsetospeed (&config, B115200);    //Baud rate
  cfsetispeed (&config, B115200);

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
  ::write(fileHandle, text.c_str(), text.length());
}

void serialPort::write(char data[], int length) {
  ::write(fileHandle, data, length);
}

string serialPort::read() {
  string input;
  int bytes = available();
  input.resize(bytes);
  ::read(fileHandle, const_cast<char*>(input.data()) , bytes);    //The const cast is less than ideal
  return(input);
}

int serialPort::available() {
  int bytes;
  ioctl(fileHandle, FIONREAD, &bytes);
  return(bytes);
}

// Updates the motor speeds according to the state variables
void serialPort::updateMotors() {
  signed char char1 = (signed char)(leftDriveSpeed);
  signed char char2 = (signed char)(rightDriveSpeed);
  signed char char3 = char1 + char2 + 1;
	signed char x[3] = {char1,char2,char3};
  if (DEBUG_TEXT){
    cout << "Sending to Arduino: " << (int)char1 << "," << (int)char2 << "," << (int)char3 << endl;
  }
  ::write(fileHandle, x, 3);
  if (DEBUG_TEXT){
    cout << "Received from Arduino: " << read() << endl;
  } 
}

void serialPort::turnLeft(int speed){
  if(speed < 0 || speed > 127)
		throw out_of_range("Motor speed must be between 0 and 127.");
  leftDriveSpeed = speed;
  rightDriveSpeed = -speed;
  updateMotors();
}

void serialPort::turnRight(int speed){
  if(speed < 0 || speed > 127)
		throw out_of_range("Motor speed must be between 0 and 127.");
  leftDriveSpeed = -speed;
  rightDriveSpeed = speed;
  updateMotors();
}

void serialPort::goForward(int speed){
  if(speed < 0 || speed > 127)
		throw out_of_range("Motor speed must be between 0 and 127.");
  leftDriveSpeed = -speed;
  rightDriveSpeed = -speed;
  updateMotors();
}

void serialPort::goBackward(int speed){
  if(speed < 0 || speed > 127)
		throw out_of_range("Motor speed must be between 0 and 127.");
  leftDriveSpeed = speed;
  rightDriveSpeed = speed;
  updateMotors();
}

void serialPort::raiseGate(int speed){
  if(speed < 0 || speed > 127)
		throw out_of_range("Motor speed must be between 0 and 127.");
  rightGateSpeed = speed;
  leftGateSpeed = speed;
  updateMotors();
}

void serialPort::holdGate(){
  rightGateSpeed = 0;
  leftGateSpeed = 0;
  updateMotors();
}

void serialPort::lowerGate(int speed){
  if(speed < 0 || speed > 127)
		throw out_of_range("Motor speed must be between 0 and 127.");
  rightGateSpeed = -speed;
  leftGateSpeed = -speed;
  updateMotors();
}

void serialPort::spinForward(int speed){
  if(speed < 0 || speed > 127)
		throw out_of_range("Motor speed must be between 0 and 127.");
  spinnerSpeed = speed;
  updateMotors();
}

void serialPort::spinBackward(int speed){
  if(speed < 0 || speed > 127)
		throw out_of_range("Motor speed must be between 0 and 127.");
  spinnerSpeed = -speed;
  updateMotors();
}

void serialPort::stopSpinner(){
  spinnerSpeed = 0;
  updateMotors();
}

void serialPort::stopMotors(){
  leftDriveSpeed = 0;
  rightDriveSpeed = 0;
  leftGateSpeed = 0;
  rightGateSpeed = 0;
  spinnerSpeed = 0;
  updateMotors();
}

serialPort::~serialPort() {
  cout << "Disconnecting from Arduino..." << endl;
  close(fileHandle);
}
