#include <Robot_Common.h>
#include <UltraSonicSensor_Library.h>
#include <OmniDrive.h>
#include <Color_Sort.h>
#include <Retriever.h>
#include <Deposit.h>

#include "MPU6050_6Axis_MotionApps20.h"



//#include <I2Cdev.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


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


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

#define INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards

#define DALLAS_SHIELD   1

#if DALLAS_SHIELD
int robotPins[4][4] = {
  {52, 50, 13, 19},
  {48, 46, 11, 18},
  {30, 28, 10, 21},
  {34, 32, 12, 20}
};

int retrieverPins[5] = {9, 8 , 7, 6, 5 };

int sorterPins[4] = {53, 51, 49, 4 };

int depositerPins[2] = { 3, 2 };

int ultrasonicPins[4][2] = {
  {45, 47},
  {25, 24},
  {27, 26},
  {29, 31}  };

#else
int robotPins[4][4] = {
  {50, 52, 5, 19},  
  {46, 48, 4, 20},
  {28, 30, 3, 21}, 
  {32, 34, 2, 18} }; 

int retrieverPins[5] = { 13, 12, 11, 10, 9 };

int sorterPins[4] = { 27, 26, 25, 8};

int depositerPins[2] = {7, 6};

int ultrasonicPins[4][2] = {
  {45, 47},
  {25, 24},
  {27, 26},
  {29, 31}  };
#endif

OmniDrive robot(robotPins);
Retriever retriever(retrieverPins);
Color_Sort sorter(sorterPins); 
Deposit depositer(depositerPins);
UltraSonic Ultrasonic(ultrasonicPins);
Timer stallTimer;

double distanceN;
double distanceE;
double distanceS;
double distanceW;

volatile bool pingTimerFlag = false;

void setup() 
{
  
    Serial3.begin(38400);
    
      // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    //initialize robot systems   
    StartUp();

    Serial.begin(115200);
}

float startYaw;

void loop() {
    update();
    /*while(1)
    {
      Serial.print("west");
      Ultrasonic.pingAverage(west);
      Serial.print("north");
      Ultrasonic.pingAverage(north);
      Serial.print("east");
      Ultrasonic.pingAverage(east);
      Serial.print("south");
      Ultrasonic.pingAverage(south);
    }*/

//ball 1
    retriever.retrieve();
    sorter.ballSort();
     
    update();
    startYaw = robot.Yaw;


//ball 2
    navigate(north, south, 50.43, east, 61.85, startYaw, true);    

    //pivot and forward adjust
    do
    {
      correctYaw(startYaw + 90);
    }while(abs((startYaw + 90) - robot.Yaw) > .5);

    //this has been adjusted to calculate differently than original algorithm
    navigate(north, east, 30, north, 49.79, startYaw + 90, false);   



//ball 3
    navigate(east, east, 30, north, 49.79, startYaw + 90, false);
    navigate(east, east, 16.11, startYaw + 90, true);

//ball 4
    navigate(west, west, 30, north, 49.79, startYaw + 90, false);
    navigate(west, west, 17.25, startYaw + 90, true);

//ball 5
    navigate(east, west, 63.9, north, 49.79, startYaw + 90, true); 

//pivot 180

    double pivot180;

    if(startYaw < 90)
        pivot180 = startYaw + 270;
    else
        pivot180 = startYaw - 90;
    
    do
    {
      correctYaw(pivot180);
    }while(abs((pivot180) - robot.Yaw) > .5);

    //ball 6
    navigate(west, west, 65.59, south, 67.24, pivot180, true);

    //ball 7 
    navigate(west, west, 25, south, 67.24, pivot180, false);
    navigate(west, west, 17.01, pivot180, true);

    //ball 8 
    navigate(east, east, 30, south, 67.24, pivot180, false);
    navigate(east, west, 114.55, pivot180, true);

    //ball 9
    navigate(west, east, 40.42, pivot180, false);
    navigate(north, south, 110, west, 89.11, pivot180, true);

    //ball 10
    navigate(west, west, 41.73, south, 110.0, pivot180, true);

    //ball 11
    navigate(west, west, 30, north, 47.75, pivot180, false);
    navigate(west, west, 16.51, pivot180, true);

    //ball 12
    navigate(east, west, 63.66, north, 47.75, pivot180, true);

    //ball 13
    navigate(east, east, 20, north, 47.75, pivot180, false);
    navigate(east, west, 112.24, pivot180, true); 

    //deposit
    sorter.findMax();
    depositer.getInitialBallColor(sorter.setBallColor());
    double pocketPivot;
    if(startYaw < 225)
      pocketPivot = startYaw + 125;
    else
      pocketPivot = startYaw - 235;

    //white deposit 1
    robot.DriveLeft(SPEED);
    while(Ultrasonic.pingAverage(west) > 96.11);
    
    do
    {
      correctYaw(pocketPivot);
    }while(abs((pocketPivot) - robot.Yaw) > .5);

    robot.DriveReverse(SPEED);
    
    if(Ultrasonic.pingAverage(south) > 0)
    {
          while(Ultrasonic.pingAverage(south) > 0); //do nothing while values are valid
    }
    else
    {
        int timeStart = millis();
        int timeEnd;
        bool exitLoopFlag = false;
        while(Ultrasonic.pingAverage(south) < 0 && !exitLoopFlag); //do nothing until values start reading
        {
          timeEnd = millis();
          if(timeEnd - timeStart > 2000)
              exitLoopFlag = true;
        }
        while(Ultrasonic.pingAverage(south) > 0 && !exitLoopFlag); //wait until robot gets too close to get good reads
    }  
    robot.BrakeAll();

    for(int i = 0; i < 7; i++)
    {
      depositer.eject(depositer.orange);
      robot.sortShake();
      delay(500);
    }
  
    //orange pocket 1
    robot.DriveForward(SPEED);
    while(Ultrasonic.pingAverage(south) < 20);
    robot.BrakeAll();

    if(startYaw < 180)
      pocketPivot = startYaw + 180;
    else
      pocketPivot = startYaw - 180;
      
    do
    {
      correctYaw(pocketPivot);
    }while(abs((pocketPivot) - robot.Yaw) > .5);

    robot.DriveForward(SPEED);
    delay(1000);
    while(Ultrasonic.pingAverage(north) > 41.73);
    robot.BrakeAll();

    if(startYaw < 325)
      pocketPivot = startYaw + 35;
    else
      pocketPivot = startYaw - 35;

    do
    {
      correctYaw(pocketPivot);
    }while(abs((pocketPivot) - robot.Yaw) > .5);

    robot.DriveReverse(SPEED);
    
    if(Ultrasonic.pingAverage(south) > 0)
    {
          while(Ultrasonic.pingAverage(south) > 0); //do nothing while values are valid
    }
    else
    {
        int timeStart = millis();
        int timeEnd;
        bool exitLoopFlag = false;
        while(Ultrasonic.pingAverage(south) < 0 && !exitLoopFlag); //do nothing until values start reading
        {
          timeEnd = millis();
          if(timeEnd - timeStart > 2000)
              exitLoopFlag = true;
        }
        while(Ultrasonic.pingAverage(south) > 0 && !exitLoopFlag); //wait until robot gets too close to get good reads
    }  
    robot.BrakeAll();

    for(int i = 0; i < 6; i++)
    {
      depositer.eject(depositer.orange);
      robot.sortShake();
      delay(500);
    }
/*
    //white deposit 2
    robot.DriveForward(SPEED);
    while(Ultrasonic.pingAverage(south) < 10);
    robot.BrakeAll();

    if(startYaw < 270)
      pocketPivot = startYaw + 90;
    else
      pocketPivot = startYaw - 90;
      
    do
    {
      correctYaw(pocketPivot);
    }while(abs((pocketPivot) - robot.Yaw) > .5);

    robot.DriveForward(SPEED);
    while(Ultrasonic.pingAverage(north) < 41.73);
    robot.BrakeAll();

    if(startYaw < 235)
      pocketPivot = startYaw + 125;
    else
      pocketPivot = startYaw - 125;

    do
    {
      correctYaw(pocketPivot);
    }while(abs((pocketPivot) - robot.Yaw) > .5);

    robot.DriveReverse(SPEED);
    
    if(Ultrasonic.pingAverage(south) > 0)
    {
          while(Ultrasonic.pingAverage(south) > 0); //do nothing while values are valid
    }
    else
    {
        while(Ultrasonic.pingAverage(south) < 0); //do nothing until values start reading
        while(Ultrasonic.pingAverage(south) > 0); //wait until robot gets too close to get good reads
    }  
    robot.BrakeAll();

    for(int i = 0; i < 3; i++)
    {
      depositer.eject(depositer.white);
      robot.sortShake();
      delay(100);
    }

    //orange deposit 2
    robot.DriveForward(SPEED);
    while(Ultrasonic.pingAverage(south) < 10);
    robot.BrakeAll();

    pocketPivot = startYaw;
      
    do
    {
      correctYaw(pocketPivot);
    }while(abs((pocketPivot) - robot.Yaw) > .5);

    robot.DriveForward(SPEED);
    while(Ultrasonic.pingAverage(north) < 41.73);
    robot.BrakeAll();

    if(startYaw < 145)
      pocketPivot = startYaw + 215;
    else
      pocketPivot = startYaw - 215;

    do
    {
      correctYaw(pocketPivot);
    }while(abs((pocketPivot) - robot.Yaw) > .5);

    robot.DriveReverse(SPEED);
    
    if(Ultrasonic.pingAverage(south) > 0)
    {
          while(Ultrasonic.pingAverage(south) > 0); //do nothing while values are valid
    }
    else
    {
        while(Ultrasonic.pingAverage(south) < 0); //do nothing until values start reading
        while(Ultrasonic.pingAverage(south) > 0); //wait until robot gets too close to get good reads
    }  
    robot.BrakeAll();

    for(int i = 0; i < 3; i++)
    {
      depositer.eject(depositer.orange);
      robot.sortShake();
      delay(100);
    }
  */  
    delay(100000000);

}

void navigate(enum direction_t driveDirection, enum direction_t primaryDirection, float primaryDistance, enum direction_t secondaryDirection, float secondaryDistance, float yaw, bool collect)
{
      //drive in primary direction to get in range
      switch(driveDirection)
      {
        case north:
          robot.DriveForward(SPEED);
          break;
        case south:
          robot.DriveReverse(SPEED);
          break;
        case east:
          robot.DriveRight(SPEED);
          break;
        case west:
          robot.DriveLeft(SPEED);
          break;
      }

    if(driveDirection != primaryDirection)
    {
      while(Ultrasonic.pingAverage(primaryDirection) < primaryDistance)
        update();
    }
    else
    {
      while(Ultrasonic.pingAverage(primaryDirection) > primaryDistance)
        update();
    }
    
    robot.BrakeAll();
    delay(100);
    //first orientation correction
    do
    {
      correctYaw(yaw);
    }while(abs((yaw) - robot.Yaw) > .5);
    
    //correct primary distance
    do
    {
      correctError(primaryDirection, primaryDistance);
    }while(abs(Ultrasonic.pingAverage(primaryDirection) - primaryDistance) > 1);
    //correct secondary distance
    do
    {
      correctError(secondaryDirection, secondaryDistance);
    }while((Ultrasonic.pingAverage(secondaryDirection) - secondaryDistance) > 1);
    
    //2nd orientation correction
    do
    {
      correctYaw(yaw);
    }while(abs((yaw) - robot.Yaw) > .5);
    //collect ball
    if(collect)
    {
      retriever.retrieve(); 
      sorter.ballSort();
    }
}

void navigate(enum direction_t driveDirection, enum direction_t primaryDirection, float primaryDistance, double yaw, bool collect)
{
        //drive in primary direction to get in range
      switch(driveDirection)
      {
        case north:
          robot.DriveForward(SPEED);
          break;
        case south:
          robot.DriveReverse(SPEED);
          break;
        case east:
          robot.DriveRight(SPEED);
          break;
        case west:
          robot.DriveLeft(SPEED);
          break;
      }
    while(Ultrasonic.pingAverage(primaryDirection) < primaryDistance)
    {
      update();
    }
    robot.BrakeAll();
    delay(100);
    //first orientation correction
    do
    {
      correctYaw(yaw);
    }while(abs((yaw) - robot.Yaw) > .5);

    //correct primary distance
    do
    {
      correctError(primaryDirection, primaryDistance);
    }while(abs(Ultrasonic.pingAverage(primaryDirection) - primaryDistance) > 1);
    //2nd orientation correction
    do
    {
      correctYaw(yaw);
    }while(abs((yaw) - robot.Yaw) > .5);
    //collect ball
    if(collect)
    {
      retriever.retrieve(); 
      sorter.ballSort();
    }
}

/*
*  StartUp
*
* Initializes the motor system by attaching interrupts to the the OP pins 
*
*/
void StartUp(void)
{
  float lastYaw = 0, currYaw = 0;
  for(int i = 0; i < 100; i++)
  {
    do
    {
      lastYaw = currYaw;
      getYaw(); 
      currYaw = robot.Yaw;
    }while(abs(currYaw - lastYaw) != 0);
  
  }
  retriever.servoSetup();
  sorter.servoSetup();
  depositer.servoSetup();
  mpu.resetFIFO();
}

void correctError(enum direction_t direction, double distance)
{
  double error = Ultrasonic.pingAverage(direction) - distance;
  double errorcompare = error;
  Serial3.print("Error: ");
  Serial3.println(error);
  while(abs(error) > .25)
  {
      if(error > 0)
        robot.drivePulse(direction);
      else
        robot.drivePulse(-direction);
     update(); 
        
      error = Ultrasonic.pingAverage(direction) - distance;
        Serial3.print("Error: ");
        Serial3.println(error);
  }
  robot.BrakeAll();
}

void correctYaw(float startYaw)
{
  update();
  Serial.print("StartRobotYaw");
  Serial.println(robot.Yaw);
  while(abs(robot.Yaw - startYaw) > .5)
  {   
      if(robot.Yaw < startYaw)
        robot.pivotPulse(robot.cw);
      else
        robot.pivotPulse(robot.ccw);
    update();
  }
  robot.BrakeAll();
  
  Serial.print("FinishYAW: ");
  update();
  Serial.println(robot.Yaw);
  
}

void getYaw(void)
{
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // wait for MPU interrupt or extra packet(s) available
    long count = 0;
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
       // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpuIntStatus & 0x02) 
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        Serial.print("yaw:\t");
        Serial.println(ypr[0] * 180/M_PI);
        
        Serial3.print("yaw:\t");
        Serial3.println(ypr[0] * 180/M_PI);
        
        robot.Yaw = (ypr[0] * 180/M_PI);
        if(robot.Yaw < 0)
            robot.Yaw += 360;
    }
}

void update(void)
{
    getYaw();
    mpu.resetFIFO();
}

void sendYaw(void)
{
       Serial3.print("Yaw: ");
       Serial3.println(robot.Yaw);
       Serial.print("Yaw: ");
       Serial.println(robot.Yaw);
}

void sendPingAverages(void)
{
  Ultrasonic.pingAverage(north);
  Ultrasonic.pingAverage(east);
  Ultrasonic.pingAverage(south);
  Ultrasonic.pingAverage(west);
}




