#include "OmniDrive.h"
/*
*
*	Robot Motor constructor
*	requires pin initialization for 4 motors
*
*	Current Settings:
*	int robotPins[4][3] = {
*	  {52, 7, 21},
*	  {50, 5, 19},
*	  {48, 6, 20},
*	  {46, 4, 18} };
*
*	@param int[3]	Motor
*
*	Motor[0] = MC 	Motor Control pin coincides with physical pin 2/15 (named pin 1A/4A) on L293 H-Bridge driver
*	Motor[1] = EN	Motor Enable pin coincides with physical pin 1/9 (named pin 1,2EN/3,4EN) on L293 H-Bridge driver
*	Motor[2] = OP	Motor Optical Encoder pin coincides with the output pin on the Optical Encoders
*
*	Each motors Optical Encoder counter is initialized to 0
*
*	pin modes for Motor Control and Motor Enable set to Outputs
*	pin mode for Optical Encoder set to Input
*
*/
OmniDrive::OmniDrive(int motorPins[4][4])
{
	//pin assignments for MotorA
	MotorA.MC1 = motorPins[0][0];
	MotorA.MC2 = motorPins[0][1];
	MotorA.EN = motorPins[0][2];
	MotorA.OP = motorPins[0][3];
	MotorA.count = 0;
	MotorA.error = 0;
	MotorA.brakeFlag = false;
	MotorA.countFlag = false;
	//pin assignments for MotorB
	MotorB.MC1 = motorPins[1][0];
	MotorB.MC2 = motorPins[1][1];
	MotorB.EN = motorPins[1][2];
	MotorB.OP = motorPins[1][3];
    MotorB.count = 0;
	MotorB.error = 0;
	MotorB.brakeFlag = false;
	MotorB.countFlag = false;
	//pin assignments for MotorC
	MotorC.MC1 = motorPins[2][0];
	MotorC.MC2 = motorPins[2][1];
	MotorC.EN = motorPins[2][2];
	MotorC.OP = motorPins[2][3];
    MotorC.count = 0;
	MotorC.error = 0;
	MotorC.brakeFlag = false;
	MotorC.countFlag = false;
	//pin assignments for MotorD
	MotorD.MC1 = motorPins[3][0];
	MotorD.MC2 = motorPins[3][1];
	MotorD.EN = motorPins[3][2];
	MotorD.OP = motorPins[3][3];
    MotorD.count = 0;
	MotorD.error = 0;
	MotorD.brakeFlag = false;
	MotorD.countFlag = false;
	//Accelerometer/Magnetometer Values
    Yaw = 0;
	//pinMode set up Motor A
	pinMode(MotorA.MC1, OUTPUT);
	pinMode(MotorA.MC2, OUTPUT);
	pinMode(MotorA.EN, OUTPUT);
	pinMode(MotorA.OP, INPUT);
	//pinMode set up Motor B
	pinMode(MotorB.MC1, OUTPUT);
	pinMode(MotorB.MC2, OUTPUT);
	pinMode(MotorB.EN, OUTPUT);
	pinMode(MotorB.OP, INPUT);
	//pinMode set up Motor C
	pinMode(MotorC.MC1, OUTPUT);
	pinMode(MotorC.MC2, OUTPUT);
	pinMode(MotorC.EN, OUTPUT);
	pinMode(MotorC.OP, INPUT);
	//pinMode set up Motor D
	pinMode(MotorD.MC1, OUTPUT);
	pinMode(MotorD.MC2, OUTPUT);
	pinMode(MotorD.EN, OUTPUT);
	pinMode(MotorD.OP, INPUT);
}
/*
*	Destructor
*/
OmniDrive::~OmniDrive()
{
}

void OmniDrive::Forward(motor_t Motor, int speed)
{
  digitalWrite(Motor.EN, LOW);
  digitalWrite(Motor.MC1, HIGH);
  digitalWrite(Motor.MC2, LOW);
  analogWrite(Motor.EN, speed);
}

void OmniDrive::Reverse(motor_t Motor, int speed)
{
	digitalWrite(Motor.EN, LOW);
	digitalWrite(Motor.MC1, LOW);
	digitalWrite(Motor.MC2, HIGH);
	analogWrite(Motor.EN, speed);
}

void OmniDrive::Brake(motor_t Motor)
{
	digitalWrite(Motor.EN, LOW);
}
/*
*	Drive Forward moves Motors A and C
*	using MC set to digital HIGH to dictate forward direction
*
*	Analog write to a speed only values 0-255 are valid
*
*/
void OmniDrive::DriveForward(int speed)
{
	digitalWrite(MotorA.EN, LOW);
	digitalWrite(MotorC.EN, LOW);
	digitalWrite(MotorA.MC1, HIGH);
	digitalWrite(MotorC.MC1, HIGH);
	digitalWrite(MotorA.MC2, LOW);
	digitalWrite(MotorC.MC2, LOW);
    //analogWrite(MotorA.EN, speed);
	//analogWrite(MotorC.EN, speed);
	for(int i = 1; i <= speed; i++)
    {
    analogWrite(MotorA.EN, speed);
	analogWrite(MotorC.EN, speed);
    }

}
/*
*	Drive Reverse moves Motors A and C
*	using MC set to digital LOW to dictate reverse direction
*
*	Analog write to a speed only values 0-255 are valid
*
*/
void OmniDrive::DriveReverse(int speed)
{
	digitalWrite(MotorA.EN, LOW);
	digitalWrite(MotorC.EN, LOW);
	digitalWrite(MotorA.MC1, LOW);
	digitalWrite(MotorC.MC1, LOW);
	digitalWrite(MotorA.MC2, HIGH);
	digitalWrite(MotorC.MC2, HIGH);
	analogWrite(MotorA.EN, speed);
	analogWrite(MotorC.EN, speed);
}
/*
*	Drive Left moves Motors B and D
*	using MC set to digital HIGH to dictate left direction
*
*	Analog write to a speed only values 0-255 are valid
*
*/
void OmniDrive::DriveLeft(int speed)
{
	digitalWrite(MotorB.EN, LOW);
	digitalWrite(MotorD.EN, LOW);
	digitalWrite(MotorB.MC1, HIGH);
	digitalWrite(MotorD.MC1, HIGH);
	digitalWrite(MotorB.MC2, LOW);
	digitalWrite(MotorD.MC2, LOW);
	analogWrite(MotorB.EN, speed);
	analogWrite(MotorD.EN, speed);
}
/*
*	Drive Right moves Motors B and D
*	using MC set to digital LOW to dictate reverse direction
*
*	Analog write to a speed only values 0-255 are valid
*
*/
void OmniDrive::DriveRight(int speed)
{
	digitalWrite(MotorB.EN, LOW);
	digitalWrite(MotorD.EN, LOW);
	digitalWrite(MotorB.MC1, LOW);
	digitalWrite(MotorD.MC1, LOW);
	digitalWrite(MotorB.MC2, HIGH);
	digitalWrite(MotorD.MC2, HIGH);
	analogWrite(MotorB.EN, speed);
	analogWrite(MotorD.EN, speed);
}

void OmniDrive::BrakeAll(void)
{
	digitalWrite(MotorA.EN, LOW);
    digitalWrite(MotorB.EN, LOW);
    digitalWrite(MotorC.EN, LOW);
	digitalWrite(MotorD.EN, LOW);
}
/*
*	Pivot Clockwise moves all Motors
*	Motors A and B move in opposite directions
*	as do Motors C and D
*	with A and C in forward drive and B and D in reverse
*
*	Analog write to a speed only values 0-255 are valid
*
*/
void OmniDrive::PivotCW(int speed)
{
	digitalWrite(MotorA.EN, LOW);
	digitalWrite(MotorB.EN, LOW);
	digitalWrite(MotorC.EN, LOW);
	digitalWrite(MotorD.EN, LOW);

	digitalWrite(MotorA.MC1, HIGH);
	digitalWrite(MotorB.MC1, LOW);
	digitalWrite(MotorC.MC1, LOW);
	digitalWrite(MotorD.MC1, HIGH);

	digitalWrite(MotorA.MC2, LOW);
	digitalWrite(MotorB.MC2, HIGH);
	digitalWrite(MotorC.MC2, HIGH);
	digitalWrite(MotorD.MC2, LOW);

    analogWrite(MotorA.EN, speed);
	analogWrite(MotorB.EN, speed);
	analogWrite(MotorC.EN, speed);
	analogWrite(MotorD.EN, speed);
}
/*
*	Pivot Counterclockwise moves all Motors
*	Motors A and B move in opposite directions
*	as do Motors C and D
*	with A and C in reverse drive and B and D in forward
*
*	Analog write to a speed only values 0-255 are valid
*
*/
void OmniDrive::PivotCCW(int speed)
{
	digitalWrite(MotorA.EN, LOW);
	digitalWrite(MotorB.EN, LOW);
	digitalWrite(MotorC.EN, LOW);
	digitalWrite(MotorD.EN, LOW);

	digitalWrite(MotorA.MC1, LOW);
	digitalWrite(MotorB.MC1, HIGH);
	digitalWrite(MotorC.MC1, HIGH);
	digitalWrite(MotorD.MC1, LOW);

	digitalWrite(MotorA.MC2, HIGH);
	digitalWrite(MotorB.MC2, LOW);
	digitalWrite(MotorC.MC2, LOW);
	digitalWrite(MotorD.MC2, HIGH);

	analogWrite(MotorA.EN, speed);
	analogWrite(MotorB.EN, speed);
	analogWrite(MotorC.EN, speed);
	analogWrite(MotorD.EN, speed);
}

void OmniDrive::drivePulse(enum direction_t direction )
{

        switch(direction)
        {
        case north:
            DriveForward(SPEED);
            break;
        case south:
            DriveReverse(SPEED);
            break;
        case east:
            DriveRight(SPEED);
            break;
        case west:
            DriveLeft(SPEED);
            break;
        default:
            break;
        }

        delay(50);
        BrakeAll();
}

void OmniDrive::pivotPulse(enum rotation_t rotation)
{
        if(rotation == cw)
            PivotCW(SPEED);
        else
            PivotCCW(SPEED);

        delay(20);
        BrakeAll();
        delay(20);
}

void OmniDrive::sortShake()
{
    DriveForward(SPEED);
    delay(100);
    DriveReverse(255);
    delay(100);
    BrakeAll();
}




