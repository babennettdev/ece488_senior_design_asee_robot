#ifndef DEPOSIT_H
#define DEPOSIT_H

#include <Servo.h>
#include <Arduino.h>
#include "Color_Sort.h"

class Deposit
{
	public:

		struct t_ServoPins
		{

			int leftPin;
			int rightPin;

		}servoPins;

		typedef enum t_Color
		{

			white,
			orange

		}ballColor;

		struct t_Deposit
		{

		    enum t_Color ballColor;
		    int count;

		}leftDeposit, rightDeposit;

        Servo leftServo;
		Servo rightServo;

		//constructor
		Deposit(int[2]);
		//destructor
		~Deposit();
		//function to initialize servos
		void servoSetup(void);
		void ejectLeft();
		void ejectRight();
		void eject(enum t_Color);
		void getColor(void);
		void getInitialBallColor(int);


        int initialBallColor;



};
#endif
