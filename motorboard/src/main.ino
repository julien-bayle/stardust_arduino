#include <ros.h>
#include "pin.h"
#include "motor.h"
#include "acs712.h"
#include "voltage.h"

/*
  StarBaby 
  Arduino Uno Board
  
  Roles :
   - Wheel motors and encoders using interrupts (INT0 and INT1)
   - Launcher motor and encoder
   - Power supply voltage and regulated voltage measurement
   - Wheel motors current measurement
   - Launcher motor current  measurement
   - Electronics current measurement 

  Motor ROS topics and messages (no control loop) :
    /joint_name/pwm (ROS -> Arduino) : PWM value from -255 to 255
    /joint_name/counter (Arduino -> ROS) : Actual encoder counter
  
  Voltage and current measures are sent as Int16 values :
    /current/name (Arduino -> ROS) : Current in mA
    /voltage/name (Arduino -> ROS) : Voltage in mV 

*/

/* Global variables */

// Motor messages are sent at 50Hz.
const int MOTOR_LOOP_RATE = 30;
const int MOTOR_INTERVAL = 1000 / MOTOR_LOOP_RATE; 
unsigned long nextMotorLoop = MOTOR_INTERVAL;

//Voltage and current messages are sent at 10Hz.
const int ADC_LOOP_RATE = 5;
const int ADC_INTERVAL = 1000 / ADC_LOOP_RATE; 
unsigned long nextADCLoop = ADC_INTERVAL;

stardust::Motor left_wheel(
    "/left_wheel/pwm",
    "/left_wheel/counter",
    LEFT_WHEEL_ENCODER, 
    LEFT_WHEEL_PWM, 
    LEFT_WHEEL_DIR,
    LEFT_WHEEL_BRAKE);
    
stardust::Motor right_wheel(
    "/right_wheel/pwm",
    "/right_wheel/counter",
    RIGHT_WHEEL_ENCODER, 
    RIGHT_WHEEL_PWM, 
    RIGHT_WHEEL_DIR,
    RIGHT_WHEEL_BRAKE);
    
stardust::Motor launcher(
    "/launcher/pwm",
    "/launcher/counter",
    LAUNCHER_ENCODER, 
    LAUNCHER_PWM);

stardust::ACS712 wheelMotorsCurrent(
    "i_wheels",
    WHEEL_MOTORS_CURRENT);

stardust::ACS712 launcherMotorCurrent(
    "i_launcher",
    LAUNCHER_MOTOR_CURRENT);
   
stardust::ACS712 electronicsCurrent(
    "i_elec",
    ELECTRONICS_CURRENT);

stardust::Voltage powerSuppyVoltage (
    "v_main",
    POWER_SUPPLY_VOLTAGE,
    5.54);

stardust::Voltage regulatedVoltage (
    "v_motors",
    REGULATED_VOLTAGE,
    5.54);

ros::NodeHandle nh;
   
/* Setup */
void setup() {
  nh.initNode();
  left_wheel.setup(&nh);
  right_wheel.setup(&nh);
  launcher.setup(&nh);
  
  wheelMotorsCurrent.setup(&nh);
  launcherMotorCurrent.setup(&nh);
  electronicsCurrent.setup(&nh);
  
  powerSuppyVoltage.setup(&nh);
  regulatedVoltage.setup(&nh);
}

/* Main loop */
void loop() {

  if(millis() > nextMotorLoop) {
    left_wheel.update();
    launcher.update();
    // Right motor should be published after left motor
    // as it triggers odom update on ROS 
    right_wheel.update();
    nextMotorLoop += MOTOR_INTERVAL;
  }
  if(millis() > nextADCLoop) {
    
    wheelMotorsCurrent.measureAndPublish();
    launcherMotorCurrent.measureAndPublish();
    electronicsCurrent.measureAndPublish();

    powerSuppyVoltage.measureAndPublish();
    regulatedVoltage.measureAndPublish();
    
    nextADCLoop += ADC_INTERVAL;
  }
  nh.spinOnce();
  delay(1);
}
