#pragma once

#include <string>
#include <termios.h>

class serialPort {
public:
	serialPort(const char* portName, int inputSpeed);
	void setupConnection();
	int set_interface_attribs (int fd,  int parity);
	void set_blocking (int fd, int should_block);
	void updateArduino();
	void turnLeft(int speed);
	void turnRight(int speed);
	void goForward(int speed);
	void goBackward(int speed);
	void drive(int left, int right);
	void stopMotors();
	void moveFlag(int pos);
	void moveGate(int pos);
	void updateLCD(std::string text);
	int getMode();
	int getButtonState();
	~serialPort();

	static const char typicalPortName[];

private:
	int fd;
	termios config;
};
