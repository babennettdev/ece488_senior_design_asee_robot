#ifndef _COMPASS_H
#define _COMPASS_H

#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include <I2Cdev.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class Compass
{
    	public:

            Compass(int);
            ~Compass();
            void Setup(void);
            void clearFifo(void);
            float getYaw(void);


            // MPU control/status vars
            bool dmpReady = false;  // set true if DMP init was successful
            uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
            uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
            uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
            uint16_t fifoCount;     // count of all bytes currently in FIFO
            uint8_t fifoBuffer[64]; // FIFO storage buffer

            // orientation/motion vars
            Quaternion q;           // [w, x, y, z]         quaternion container
            VectorInt16 aa;         // [x, y, z]            accel sensor measurements
            VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
            VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
            VectorFloat gravity;    // [x, y, z]            gravity vector
            float euler[3];         // [psi, theta, phi]    Euler angle container
            float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

            volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

            // class default I2C address is 0x68
            // specific I2C addresses may be passed as a parameter here
            // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
            // AD0 high = 0x69
            MPU6050 mpu;

};

#endif
