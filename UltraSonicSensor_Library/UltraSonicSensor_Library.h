/*
    Ultra Sonic Distance Sensor Library for Arduino Mega
    Purpose: Provide easy to use set-up for an HC-SR04 ultrasonic distance sensor
    with built in functions for pulse/echo and conversion from microseconds (uS)
    to centimeters (cm).

    For ASEE Competition 2018 - WorkerBots (IUPUI ECE Senior Design)

    Created by Blake Bennett
    Updated: 3/13/2018

*/

#ifndef ULTRASONICSENSOR_LIB
#define ULTRASONICSENSOR_LIB

#include "Robot_Common.h"

#define uS_per_cm 58
#define AVERAGECOUNT 12
#define DEBUG 1

class UltraSonic
{
public:
/*
    enum direction_t
    {
        north = 0,
        east,
        south,
        west
    };*/
    struct sensor_t
    {

        int pin_trigger_out;
        int pin_echo_in;
        unsigned int triggerDelay;

    }North, East, South, West;

    int pin_trigger_out;
    int pin_echo_in;
    unsigned int triggerDelay;
    //int echo_time_uS
    //int distance_cm

    UltraSonic(int pin_trigger_out, int pin_echo_in);
    UltraSonic(int[4][2]);
    ~UltraSonic();

    double ping(void);
    double ping(struct sensor_t);
    double timeToDistance(long echo_time_uS);
    double pingAverage(enum direction_t);
    double pingAverage(enum direction_t, int);



};
#endif // ULTRASONICSENSOR_LIB
