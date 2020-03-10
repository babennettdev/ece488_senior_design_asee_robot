#ifndef OMNIDRIVE_H
#define OMNIDRIVE_H

#include <Arduino.h>
#include <Timer.h>
#include "Robot_Common.h"


#define SPEED 112
#define PIVOT_LOW_SPEED 80
#define DRIVE_LOW_SPEED 90

class OmniDrive{

	public:

	struct motor_t
	{
		int MC1;
		int MC2;
		int EN;
		int OP;
		int count;
		int error;
		bool brakeFlag;
		bool countFlag;

	}MotorA, MotorB, MotorC, MotorD;

    float Yaw;
    int pivot_low_speed = PIVOT_LOW_SPEED;
    int drive_low_speed = DRIVE_LOW_SPEED;
    Timer clickTimer;
	enum rotation_t
	{
	    cw,
	    ccw
	};

		OmniDrive(int motorPins[4][4]);
		~OmniDrive();
		//Individual Motor Directional Movement functions
		void Forward(motor_t Motor, int speed);
		void Reverse(motor_t Motor, int speed);
		void Brake(motor_t Motor);
		//Multiple Motor Directional Movement functions
		void DriveForward(int speed);
		void DriveReverse(int speed);
		void DriveLeft(int speed);
		void DriveRight(int speed);
		void PivotCW(int speed);
		void PivotCCW(int speed);
		void BrakeAll(void);
		//Motor synchronization functions
		void drivePulse(enum direction_t);
		void pivotPulse(enum rotation_t);
		void sortShake(void);




};
#endif

