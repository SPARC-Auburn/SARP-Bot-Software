#pragma once

#include <string>
#include <termios.h>

class serialPort {
public:
	serialPort(const char* portName);
	
	void write(std::string text);
	template<int size>
	void write(char data[size]) { write(data, size); }
	
	int available();
	void setupConnection();
	std::string read();
	std::string updateArduino();
	void turnLeft(int speed);
	void turnRight(int speed);
	void goForward(int speed);
	void goBackward(int speed);
	void moveGate(int pos);
	void moveFlag(int pos);
	void updateLCD(std::string text);
	void drive(int left,int right);
	int getMode();
	int getButtonState();
	void stopMotors();
	~serialPort();

	static const char typicalPortName[];

private:
	int fileHandle;
	termios config;
	
	void write(char data[], int length);		//Only exists so that template's ::write call does not have to go in the header
};
