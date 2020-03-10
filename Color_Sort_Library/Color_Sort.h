#ifndef COLOR_SORT_H
#define COLOR_SORT_H

#include <Arduino.h>
#include <Servo.h>

#define REST 85
#define LEFT 0
#define RIGHT 180

#define VARIANCE 10
#define AVERAGECOUNT 3


class Color_Sort
{
	public:

	//	Sorter Arduino Pinout structure
	struct t_SorterPins
	{
		int S2;
		int S3;
		int sensorOut;
		int sortServo;

	}colorSortPins;

	//	enumeration for rgb colors
	enum t_Color
	{
		red,
		green,
		blue
	};
	//	enumeration for ball color
	enum t_ballColor
	{
		white = 1,
		orange = 2
	};
	//	enumeration for sort side
	enum t_sortSide
	{
		left = 1,
		right = 2
	};
	// 	enumeration for color validity
	enum t_colorVariation
	{
		out_of_range,
		in_range,
		different_color
	};

    //	rgb color storage structure
	struct t_RGB
	{
		int red;
		int green;
		int blue;
		enum t_sortSide sortSide;
		enum t_ballColor ballColor;
	} rgb, initialBall, otherBall, OOR;

	//Servo object
	Servo SortServo;

		//constructor
		Color_Sort(int sorterPins[4]);
		//destructor
		~Color_Sort();
		void servoSetup(void);
		void ballSort(void);
		void printColors(void);
		int setBallColor(void);

		void colorRead(void);
		void rgbAverage(struct t_RGB);
		void errorCheck(void);
		int rgbCheck(void);
		int colorCheck(int, enum t_Color);
		void Sort(int);
		void findMax(void);
		void calibrate(void);

		bool twoTypesFlag;
		bool initialOverflowFlag;
		bool otherOverflowFlag;
		int initialCt;
		int otherCt;
		int oorCt;

};
#endif
