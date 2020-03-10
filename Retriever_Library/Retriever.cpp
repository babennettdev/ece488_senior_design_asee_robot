#include "Retriever.h"

/*
*
*		Retriever Constructor
*
*	takes in an array as pins for the 5 servos on the module
*
*			Lever Left Servo
*			Lever Right Servo
*			Rotate Left Servo
*			Rotate Right Servo
*			Gripper Servo
*
*/
Retriever::Retriever(int pins[5])
{
	retrieverPins.lever_right = pins[0];
	retrieverPins.lever_left = pins[1];
	retrieverPins.rotate_left = pins[2];
	retrieverPins.rotate_right = pins[3];
	retrieverPins.gripper = pins[4];
}

Retriever::~Retriever(void)
{
}

Retriever::servoSetup(void)
{
	//attach servo Pins to Servo objects
	leftLeverServo.attach(retrieverPins.lever_left);
	rightLeverServo.attach(retrieverPins.lever_right);
	leftRotateServo.attach(retrieverPins.rotate_left);
	rightRotateServo.attach(retrieverPins.rotate_right);
	gripperServo.attach(retrieverPins.gripper);
	//write Servos to initial positions
	raiseLever();
	rotateGripper(1);
	openGripper();
}

Retriever::retrieve(void)
{
    lowerLever();
    openGripper();
    rotateGripper(0);
    delay(500);
    closeGripper();
    delay(500);
    rotateGripper(1);
    raiseLever();
    openGripper();
    delay(500);
}

Retriever::raiseLever(void)
{

    for(int i = 95; i <= 180; i++)
  {
    leftLeverServo.write(i);
    rightLeverServo.write(180 - i);
    delay(20);
  }

}

Retriever::lowerLever(void)
{
  for(int i = 180; i > 95; i--)
  {
    leftLeverServo.write(i);
    rightLeverServo.write(180 - i);
    delay(20);
  }
}

Retriever::rotateGripper(bool rotate)
{
	if(rotate)
	{
	    for(int i = 95; i <= 180; i++)
		{
		    leftRotateServo.write(i);
            rightRotateServo.write(180- i);
            delay(5);
		}
	}
	else
	{
	    for(int i = 180; i >= 95; i--)
		{
            leftRotateServo.write(i);
            rightRotateServo.write(180 - i);
            delay(5);
		}
	}
}

Retriever::openGripper(void)
{
	gripperServo.write(0);
}

Retriever::closeGripper(void)
{
	gripperServo.write(70);
}
