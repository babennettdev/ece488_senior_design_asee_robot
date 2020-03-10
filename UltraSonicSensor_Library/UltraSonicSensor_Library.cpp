/*
    Ultra Sonic Distance Sensor Library for Arduino Mega
    Purpose: Provide easy to use set-up for an HC-SR04 ultrasonic distance sensor
    with built in functions for pulse/echo and conversion from microseconds (uS)
    to centimeters (cm).

    For ASEE Competition 2018 - WorkerBots (IUPUI ECE Senior Design)

    Created by Blake Bennett
    Updated: 3/13/2018

*/

#include "UltraSonicSensor_Library.h"

UltraSonic::UltraSonic(int trigger, int echo)
{

    pin_trigger_out = trigger;
    pin_echo_in = echo;
    triggerDelay = 0;
    pinMode(pin_trigger_out, OUTPUT);
    pinMode(pin_echo_in, INPUT);

}

UltraSonic::UltraSonic(int pins[4][2])
{
    North.pin_trigger_out = pins[0][0];
    North.pin_echo_in = pins[0][1];
    North.triggerDelay = 0;

    East.pin_trigger_out = pins[1][0];
    East.pin_echo_in = pins[1][1];
    East.triggerDelay = 0;

    South.pin_trigger_out = pins[2][0];
    South.pin_echo_in = pins[2][1];
    South.triggerDelay = 0;

    West.pin_trigger_out = pins[3][0];
    West.pin_echo_in = pins[3][1];
    West.triggerDelay = 0;

    pinMode(North.pin_trigger_out, OUTPUT);
    pinMode(North.pin_echo_in, INPUT);

    pinMode(East.pin_trigger_out, OUTPUT);
    pinMode(East.pin_echo_in, INPUT);

    pinMode(South.pin_trigger_out, OUTPUT);
    pinMode(South.pin_echo_in, INPUT);

    pinMode(West.pin_trigger_out, OUTPUT);
    pinMode(West.pin_echo_in, INPUT);

}

UltraSonic::~UltraSonic()
{
    //EMPTY
}

double UltraSonic::ping(void)
{

    digitalWrite(pin_trigger_out, LOW);
    triggerDelay = micros();
    while(micros() < (triggerDelay+2)); //wait 2 uS for trigger pin
    digitalWrite(pin_trigger_out, HIGH);
    triggerDelay = micros();   //change triggerDelay for HIGH pulse
    while(micros() < (triggerDelay+10));    //wait 10 uS for trigger pin
    digitalWrite(pin_trigger_out, LOW);
    long echo_time = pulseIn(pin_echo_in, HIGH, 15000); //TIME OUT AFTER 15,000 uS (15 ms)

    return timeToDistance(echo_time); //return distance in cm

}

double UltraSonic::ping(struct sensor_t sensor)
{

    digitalWrite(sensor.pin_trigger_out, LOW);
    sensor.triggerDelay = micros();
    while(micros() < (sensor.triggerDelay+2)); //wait 2 uS for trigger pin
    digitalWrite(sensor.pin_trigger_out, HIGH);
    sensor.triggerDelay = micros();   //change triggerDelay for HIGH pulse
    while(micros() < (sensor.triggerDelay+10));    //wait 10 uS for trigger pin
    digitalWrite(sensor.pin_trigger_out, LOW);
    long echo_time = pulseIn(sensor.pin_echo_in, HIGH, 15000); //TIME OUT AFTER 15,000 uS (15 ms)

    return timeToDistance(echo_time); //return distance in cm

}

double UltraSonic::timeToDistance(long echo_time_uS)
{
    if (echo_time_uS == 0)
    {
        return -1;  //Error due to timeout of pulseIn()
    }
    else
    {
        return (echo_time_uS * 0.034 / 2);
    }
}

double UltraSonic::pingAverage(enum direction_t direction)
{
    double distance = 0;
    double total = 0;
    int ping_count = 0;
    double Avg = 0;


     for(ping_count; ping_count < AVERAGECOUNT; ping_count++)
     {
        switch(direction)
        {
        case north:
            distance = ping(North);
            break;
        case east:
            distance = ping(East);
            break;
        case south:
            distance = ping(South);
            break;
        case west:
            distance = ping(West);
            break;
        }
        total += distance;
     }

#if DEBUG
        switch(direction)
        {
        case north:
            Serial3.print("North ");
            break;
        case east:
            Serial3.print("East ");
            break;
        case south:
            Serial3.print("South ");
            break;
        case west:
            Serial3.print("West ");
            break;
        }
        Serial.print("Average: ");
        Serial.println(total / ping_count);
#endif
        return total / ping_count;
}

double UltraSonic::pingAverage(enum direction_t direction, int averageCount)
{
    double distance = 0;
    double total = 0;
    int ping_count = 0;
    double Avg = 0;


     for(ping_count; ping_count < averageCount; ping_count++)
     {
        switch(direction)
        {
        case north:
            distance = ping(North);
            break;
        case east:
            distance = ping(East);
            break;
        case south:
            distance = ping(South);
            break;
        case west:
            distance = ping(West);
            break;
        }
        total += distance;
     }

#if DEBUG
        switch(direction)
        {
        case north:
            Serial3.print("North ");
            break;
        case east:
            Serial3.print("East ");
            break;
        case south:
            Serial3.print("South ");
            break;
        case west:
            Serial3.print("West ");
            break;
        }
        Serial3.print("Average: ");
        Serial3.println(total / ping_count);
#endif
        return total / ping_count;
}
