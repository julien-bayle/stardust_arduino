/* Pin definition (0 & 1 are used by TX/RX) */

const byte LEFT_WHEEL_ENCODER = 2;      // HALL SENSOR ENCODER
const byte RIGHT_WHEEL_ENCODER = 3;     // HALL SENSOR ENCODER
const byte LAUNCHER_ENCODER = 4;        // HALL SENSOR ENCODER

const byte LAUNCHER_PWM = 5;            // MOTOR (timer 0 / output 5 & 6)

const byte RIGHT_WHEEL_PWM = 6;         // MOTOR ( timer 0 / output 5 & 6)
const byte RIGHT_WHEEL_DIR = 7;         // MOTOR
const byte RIGHT_WHEEL_BRAKE = 8;       // MOTOR

const byte LEFT_WHEEL_DIR = 9;          // MOTOR
const byte LEFT_WHEEL_BRAKE = 10;       // MOTOR
const byte LEFT_WHEEL_PWM = 11;         // MOTOR ( timer 1 / output 11 & 12)

const byte LAUNCHER_MOTOR_CURRENT = 0;  // ADC
const byte WHEEL_MOTORS_CURRENT = 1;    // ADC
const byte POWER_SUPPLY_VOLTAGE = 2;    // ADC
const byte REGULATED_VOLTAGE = 3;       // ADC
const byte ELECTRONICS_CURRENT = 4;     // ADC
