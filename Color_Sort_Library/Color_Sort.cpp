#include "Color_Sort.h"
/*
*	Constructor for Color_Sort Module
*	uses a TCS3200 Light to Color Frequency module
*
*	Set the frequency scaling to 20%, this isn't dynamically adjustable in system
*	Calibration of outputs is set with this scaling pre-determined
*
*	S0 connected to Vcc
*	S1 connected to ground
*
*	Pins S2 and S3 choose which color filter is activated
*
*	S2	S3
*	L	L	Red
*	L	H	Blue
*	H	L	Clear (no filter)
*	H	H	Green
*
*
*	Servo pin is attached to Servo member object named SortServo
*	Color Servo rests @ 90 degree angle
*	White Sort goes to 180 degrees
*	Orange Sort goes to 0 degrees
*
*/
Color_Sort::Color_Sort(int sorterPins[4])
{
	//pin initializiation
	colorSortPins.S2 = sorterPins[0];
	colorSortPins.S3 = sorterPins[1];
	colorSortPins.sensorOut = sorterPins[2];
	colorSortPins.sortServo = sorterPins[3];
	//rgb value initialization
	 rgb.red = 0;
	 rgb.green = 0;
	 rgb.blue = 0;
	 initialBall.red = 0;
	 initialBall.green = 0;
	 initialBall.blue = 0;
	 initialBall.sortSide = 0;
	 initialBall.ballColor = 0;
	 initialCt = 0;
	 initialOverflowFlag = false;
	 otherBall.red = 0;
	 otherBall.green = 0;
	 otherBall.blue = 0;
	 otherBall.sortSide = 0;
	 otherBall.ballColor = 0;
	 otherCt = 0;
	 otherOverflowFlag = false;

	 OOR.red = 0;
	 OOR.green = 0;
	 OOR.blue = 0;
	 OOR.sortSide = 0;
	 OOR.ballColor = 0;
	 oorCt = 0;
	//pin mode set up
	pinMode(colorSortPins.S2, OUTPUT);
	pinMode(colorSortPins.S3, OUTPUT);
	pinMode(colorSortPins.sensorOut, INPUT);
	Serial.begin(9600);
}
//destructor
Color_Sort::~Color_Sort()
{
}

void Color_Sort::servoSetup(void)
{
		SortServo.attach(colorSortPins.sortServo);
		SortServo.write(REST);
		calibrate();
}

/*
*	Arduino hardware function to set appropriate pins
*	and get appropriate reads from pins to determined
*	color values of the ping pong ball to be sorted
*
*/
void Color_Sort::colorRead(void)
{
//setting red filtered diodes to be read
digitalWrite(colorSortPins.S2, LOW);
digitalWrite(colorSortPins.S3, LOW);

//reading output frequency
rgb.red = pulseIn(colorSortPins.sensorOut, LOW);
//TO DO: implement timer and not delay
delay(100);

//setting green filtered diodes to be read
digitalWrite(colorSortPins.S2, HIGH);
digitalWrite(colorSortPins.S3, HIGH);

//reading output frequency
rgb.green = pulseIn(colorSortPins.sensorOut, LOW);
//TO DO: implement timer and not delay
delay(100);

//setting blue filtered diodes to be read
digitalWrite(colorSortPins.S2, LOW);
digitalWrite(colorSortPins.S3, HIGH);

//reading output frequency
rgb.blue = pulseIn(colorSortPins.sensorOut, LOW);
//TO DO: implement timer and not delay
delay(100);
}

void Color_Sort::rgbAverage(struct t_RGB color)
{
        int redAverage = 0;
        int greenAverage = 0;
        int blueAverage = 0;

        for(int i = 0; i < AVERAGECOUNT; i++)
        {
            colorRead();
            redAverage += rgb.red;
            greenAverage += rgb.green;
            blueAverage += rgb.blue;
        }

        color.red = redAverage / AVERAGECOUNT;
        color.green = greenAverage / AVERAGECOUNT;
        color.blue = blueAverage / AVERAGECOUNT;

}

void Color_Sort::ballSort(void)
{
    rgbAverage(rgb);

	if(initialCt + otherCt == 0)
	{

		if(rgbCheck() == out_of_range)
        {
            Serial.println("Out of Range");
            	errorCheck();
        }

		else
		{
            initialBall.red = rgb.red;
			initialBall.green = rgb.green;
			initialBall.blue = rgb.blue;
			initialBall.sortSide = left;
			Sort(LEFT);
			initialCt++;
		}
	}
	else if(otherCt == 0 && rgbCheck() == different_color)
	{
        printColors();
		otherBall.red = rgb.red;
		otherBall.green = rgb.green;
		otherBall.blue = rgb.blue;
		otherCt++;
		Sort(RIGHT);
	}

	else
	{
		switch(rgbCheck())
		{
			case in_range:
			    Serial.println("Another Initial");
                //if sorted as in range but already full sets overflow flag
                if(initialCt >= 7)
                {
                   initialOverflowFlag = true;
                   otherCt++;
                }

				else
				initialCt++;

				Sort(LEFT);
				break;

			case different_color:
			    Serial.println("Another different");
			    printColors();
                if(otherCt >= 7)
                {
                    otherOverflowFlag = true;
                    initialCt++;
                }

				else
				otherCt++;

				Sort(RIGHT);
				break;

			case out_of_range:
			    Serial.println("Out of Range");
			    Sort(LEFT);
				//errorCheck();
				break;
		}
	}

}
/*
*       re evaluates the rgb values if original was out of range
*		marks balls as improperly read if no value is found
*
*/
void Color_Sort::errorCheck()
{
	//int errorCount = 0;

	while(rgbCheck() == out_of_range && oorCt < 1)
	{
		colorRead();
		if(rgbCheck() == out_of_range)
        {
           Serial.println("Error check still bad read");
        }
		else
        {
            Serial.println("Found a good read");
            ballSort();
        }
        oorCt++;
	}
}

void Color_Sort::calibrate(void)
{
    colorRead();
    OOR.red = rgb.red;
    OOR.green = rgb.green;
    OOR.blue = rgb.blue;
}

/*
*		returns color select for all 3 rgb selections
*		each must return a value in range to be valid
*
*		0 for out of range
*		1 for white
*		2 for orange
*
*/
int Color_Sort::rgbCheck()
{
    //any color returns out of range
    if(colorCheck(rgb.red, red) == out_of_range || colorCheck(rgb.green, green) == out_of_range || colorCheck(rgb.blue, blue) == out_of_range)
        return out_of_range;
    //all colors are in range
    else if(colorCheck(rgb.red, red) == in_range && colorCheck(rgb.green, green) == in_range && colorCheck(rgb.blue, blue) == in_range)
        return in_range;
    //all colors are in range or a different color
    else if(colorCheck(rgb.red, red) >= in_range && colorCheck(rgb.green, green) >= in_range
		&& colorCheck(rgb.blue, blue) >= in_range)
        return different_color;
}

/*
*       returns color select
*
*       0 for out of range
*       1 for match to first ball
*       2 for match to !first ball
*
*/
int Color_Sort::colorCheck(int colorVal, enum t_Color color)
{
	switch(color)
	{
		case red:
			    if(colorVal <= initialBall.red + VARIANCE && colorVal >= initialBall.red  - VARIANCE)
					return in_range;
				else if (colorVal >= OOR.red - VARIANCE)
					return out_of_range;
				else
					return different_color;
					break;
        case green:
				if(colorVal <= initialBall.green + VARIANCE && colorVal >= initialBall.green  - VARIANCE)
					return in_range;
				else if(colorVal >= OOR.green - VARIANCE)
					return out_of_range;
				else
					return different_color;
					break;
		case blue:
				if(colorVal <= initialBall.blue + VARIANCE && colorVal >= initialBall.blue  - VARIANCE)
					return in_range;
				else if(colorVal >= OOR.blue - VARIANCE)
					return out_of_range;
				else
					return different_color;
					break;
	}
}

void Color_Sort::Sort(int Direction)
{
    if(initialOverflowFlag)
    {
        if(Direction == LEFT)
        SortServo.write(RIGHT);
    }
    else if(otherOverflowFlag)
    {
        if(Direction == RIGHT)
        SortServo.write(LEFT);
    }
    else
    SortServo.write(Direction);

    delay(500);
    SortServo.write(REST);
}

void Color_Sort::findMax(void)
{
    int initialMax = initialBall.red;
    int otherMax = otherBall.red;

    if(initialBall.green > initialMax)
        initialMax = initialBall.green;
    else if(initialBall.blue > initialMax)
        initialMax = initialBall.blue;

    if(otherBall.green > otherMax)
        otherMax = otherBall.green;
    else if(otherBall.blue > otherMax)
        otherMax = otherBall.blue;

    if(initialMax > otherMax)
    {
        initialBall.ballColor = orange;
        otherBall.ballColor = white;
    }
    else
    {
        initialBall.ballColor = white;
        otherBall.ballColor = orange;
    }
}

int Color_Sort::setBallColor(void)
{
    return initialBall.ballColor;
}

void Color_Sort::printColors(void)
{
    Serial.print("Red: ");
Serial.println(rgb.red);
Serial.print("Green: ");
Serial.println(rgb.green);
Serial.print("Blue: ");
Serial.println(rgb.blue);
}

