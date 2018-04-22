#include <ros.h>
#include "pin.h"
#include "sharp_ir.h"
#include "servo.h"
#include "sonar.h"

/*
  StarBaby 
  Arduino Nano Board
  
  Roles :
   - Two servomotors
   - Two SHARP distance measurements
   - Three sonar sensors (HC-SR04)

  ROS topics :
    /servo/front (ROS -> Arduino) : Int8 value from 0 to 255
    /servo/launcher (ROS -> Arduino) : Int8 value from 0 to 255
    /sharp/name (Arduino -> ROS) : sensor_msgs/Range
    /sonar/name (Arduino -> ROS) : sensor_msgs/Range
*/

/* Global variables */

// Range messages are sent at 20Hz.
const int RANGE_LOOP_RATE = 20;
const int RANGE_INTERVAL = 1000 / RANGE_LOOP_RATE; 
unsigned long nextRangeLoop = RANGE_INTERVAL;

// Only one ultrasound sensor are read for each loop
unsigned int sensorIterator = 0; 

stardust::Servomotor frontServo(
    "/servo/front",
    SERVOMOTOR_FRONT,
    159);

stardust::Servomotor launcherServo(
    "/servo/launcher",
    SERVOMOTOR_LAUNCHER,
    170);

stardust::SHARP_GP2Y0A51SK0F sideRange(
    "/range/side",
    SHARP_SIDE);

stardust::SHARP_GP2Y0A51SK0F launcherRange(
    "/range/launcher",
    SHARP_LAUNCHER);

stardust::SonarHcSr04 leftSonar(
    "/sonar/left",
    ULTRASOUND_ECHO_LEFT,
    ULTRASOUND_TRIG_LEFT);

stardust::SonarHcSr04 rightSonar(
    "/sonar/right",
    ULTRASOUND_ECHO_RIGHT,
    ULTRASOUND_TRIG_RIGHT);

stardust::SonarHcSr04 centerSonar(
    "/sonar/center",
    ULTRASOUND_ECHO_CENTER,
    ULTRASOUND_TRIG_CENTER);

ros::NodeHandle nh;
   
/* Setup */
void setup() {
  nh.initNode();
  frontServo.setup(&nh);
  launcherServo.setup(&nh);

  sideRange.setup(&nh);
  launcherRange.setup(&nh);

  leftSonar.setup(&nh);
  rightSonar.setup(&nh);
  centerSonar.setup(&nh);
}

/* Main loop */
void loop() {
  if(millis() > nextRangeLoop) {
    sideRange.measureAndPublish(&nh);
    launcherRange.measureAndPublish(&nh);

    switch (sensorIterator) {
      case 0: leftSonar.measureAndPublish(&nh); break;
      case 1 : rightSonar.measureAndPublish(&nh); break;
      default : centerSonar.measureAndPublish(&nh); break;
    }
    
    sensorIterator = (sensorIterator + 1) % 3;
    nextRangeLoop += RANGE_INTERVAL;
  }
  nh.spinOnce();
  delay(1);
}
