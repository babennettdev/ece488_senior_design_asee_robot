# Indiana University - Purdue University Indianapolis Electrical and Computer Engineering Senior Design 2018
## ECE487 and ECE488
## Blake Bennett

![WorkerBot Demo](ECE488_ASEE_Robotics_Demonstration.gif)

### Purpose
The purpose of this project was to design and program a robot to the specifications of the 2018 ASEE Robotics Competition "WorkerBots." As upperclassmen, we were prevented from entering the competition, but the goal was to work through the design process, document it, document lessons learned, and create a lesson plan to teach underclassmen the skills necessary to be successful in future robotics competitions. 

Rules for the 2018 ASEE Robotics Competition can be found here: http://faculty.tcc.edu/PGordy/ASEE/ASEE2018/ASEE%20TYCD%202018%20Rules%20%20(6-21-17).pdf

### Role
I played many roles in the design of this project.
 
  * Wrote the UltraSonicSensor library and lead the design of that porition of the navigation.
  * Debugged ultrasonic sensor readings (very inconsistent on our homemade competition board). 
  * Co-wrote the OmniDrive library and assisted in finetuning rotary encoder target values for pivoting.
  * Wrote a Bluetooth Arduino driver (not currently in this repository) to control the robot manually and assist in debugging the ultrasonic sensors and inertial measurement unit (IMU) and provide accurate measurements at key positions.
  * Assisted in the implementation of the MPU6050 library for the IMU. 
  * Lead documentation of the entirety of the project.
  * Wrote ten lesson plan outlines for underclassmen students detailing the key points in designing an Arduino-based robot.
  * Assisted the team in all hardware/software/mechanical design.
  * Designed prototype 3D models for different parts of the robot.

### Documentation
The final design document, as well as our final presentation and poster are available in this repository. Ten lesson outlines for underclassmen are located in the appendix of the design document.

### Result
The robot, admittedly, would've fared poorly at the ASEE Robotics Competition had we been able to compete. We learned a lot of lessons in the mechanical design of the robot and had we had the time to redesign the structure of the robot, picking and sorting the balls would have been much quicker and easier. Our software algorithms were spot-on for the final design of the robot, but poor sensor measurements and an inaccurate robotic arm resulted in slow and, at times, inconsistent completion times and score. But, the goal of this project was to learn from our failures and from this project the entire team gained a great deal of experience in software design, hobby electronics, 3D modeling, and robotic design. Our final documentation and lessons no doubt will be beneficial for student who wish to succeed in any future robotic competition. 
