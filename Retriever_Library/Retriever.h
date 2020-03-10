#ifndef RETIREVER_H
#define RETRIEVER_H

#include <Servo.h>
#include <Arduino.h>


class Retriever
{

	public:
		//pin names
		struct t_Pins
		{

			int lever_left;
			int lever_right;
			int rotate_left;
			int rotate_right;
			int gripper;

		}retrieverPins;
		//Servos
		Servo leftLeverServo;
		Servo rightLeverServo;
		Servo leftRotateServo;
		Servo rightRotateServo;
		Servo gripperServo;

		//constructor
		Retriever(int[4]);
		//destructor
		~Retriever(void);
		servoSetup(void);
		retrieve(void);
		raiseLever(void);
		lowerLever(void);
		rotateGripper(bool);
		openGripper(void);
		closeGripper(void);

};
#endif
