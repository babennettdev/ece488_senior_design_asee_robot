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
			int pivot;
			int gripper;
		
		}retrieverPins;
		//Servos
		Servo leftServo;
		Servo rightServo;
		Servo pivotServo;
		Servo gripperServo;

		//constructor
		Retriever(int[4]);
		//destructor
		~Retriever(void);
		servoSetup(void);
		raiseLever(void);
		lowerLever(void);
		pivotGripper(bool);
		openGripper(void);
		closeGripper(void);
		
		
	


};
#endif