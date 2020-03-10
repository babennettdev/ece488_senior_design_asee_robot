#include "Deposit.h"

//constructor
Deposit::Deposit(int depositerPins[2])
{
	servoPins.leftPin = depositerPins[0];
	servoPins.rightPin = depositerPins[1];
	leftDeposit.count = 0;
	leftDeposit.ballColor = white;
	rightDeposit.count = 0;
	rightDeposit.ballColor = orange;
	initialBallColor = 0;
}
//destructor
Deposit::~Deposit()
{
}
// servo Setup function attaches pins and writes to a start angle
void Deposit::servoSetup()
{
	leftServo.attach(servoPins.leftPin);
	rightServo.attach(servoPins.rightPin);
	leftServo.write(180);
	rightServo.write(120);
}

void Deposit::getColor(void)
{
    if(initialBallColor == white)
    {
        leftDeposit.ballColor == white;
        rightDeposit.ballColor == orange;
    }
    else
    {
        leftDeposit.ballColor == orange;
        rightDeposit.ballColor == white;
    }
}

void Deposit::eject(enum t_Color color)
{

    if(leftDeposit.ballColor == color)
            ejectLeft();

    else if(rightDeposit.ballColor == color)
            ejectRight();

}

void Deposit::ejectLeft()
{
		leftServo.write(120);
		delay(1000);
		leftServo.write(180);
		delay(1000);
}

void Deposit::ejectRight()
{
		rightServo.write(180);
		delay(1000);
		rightServo.write(120);
		delay(1000);
}

void Deposit::getInitialBallColor(int color)
{
    initialBallColor = color;
    getColor();
}

