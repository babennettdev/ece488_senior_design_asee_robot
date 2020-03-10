#include "Retriever.h"

/*
*
*		Retriever Constructor
*
*	takes in an array as pins for the 4 servos on the module
*
*			Lever Right Servo
*			Lever Left Servo
*			Pivot Servo
*			Gripper Servo
*
*/
Retriever::Retriever(int pins[4])
{
	retrieverPins.lever_right = pins[0];
	retrieverPins.lever_left = pins[1];
	retrieverPins.pivot = pins[2];
	retrieverPins.gripper = pins[3];
}

Retriever::~Retriever(void)
{
}

Retriever::servoSetup(void)
{
	//attach servo Pins to Servo objects
	leftServo.attach(retrieverPins.lever_left);
	leftServo.attach(retrieverPins.lever_right);
	pivotServo.attach(retrieverPins.pivot);
	gripperServo.attach(retrieverPins.gripper);
	//write Servos to initial positions
	leftServo.write(130);
	rightServo.write(30);
	pivotServo.write(180);
	gripperServo.write(180);
}

Retriever::raiseLever(void)
{
	leftServo.write(130);
	rightServo.write(30);
}

Retriever::lowerLever(void)
{
  for(int i = 130; i > 40; i--)
  {
    leftServo.write(i);
    rightServo.write(160 - i);
    delay(30);
  }
}

Retriever::pivotGripper(bool direction)
{
	if(direction)
		pivotServo.write(180);
	else
		pivotServo.write(0);
}

Retriever::openGripper(void)
{
	gripperServo.write(100);
}

Retriever::closeGripper(void)
{
	gripperServo.write(180);
}
