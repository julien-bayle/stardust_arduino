#ifndef _STARDUST_MOTOR_h
#define _STARDUST_MOTOR_h
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <math.h>

namespace stardust
{
  /* Encoder counter (For ISR routine) */
  int encoder_counter [2] = {0, 0};

  /* Encoder interrrupt methods */
  void ISR_0() { encoder_counter[0]++; }
  void ISR_1() { encoder_counter[1]++; }

  class Motor
  {
    private:
    
      int pwm = 0;                   // PWM sent by ROS
      long last_counter_refresh = 0; // Last counter refresh

      const byte encoder_pin;
      const byte pwm_pin;
      const byte dir_pin;
      const byte brake_pin;

      std_msgs::Int16 counter;
      ros::Subscriber<std_msgs::Float64, Motor> sub_pwm;
      ros::Publisher pub_counter;

    public :

      Motor(const char *_pwm_topic_name,
        const char *_counter_topic_name,
        const byte _encoder_pin, 
        const byte _pwm_pin, 
        const byte _dir_pin = -1,
        const byte _brake_pin = -1):
          encoder_pin(_encoder_pin),
          pwm_pin(_pwm_pin),
          dir_pin(_dir_pin),
          brake_pin(_brake_pin),
          sub_pwm(_pwm_topic_name, &Motor::updatePWM, this ),
          pub_counter(_counter_topic_name, &counter) { }
    
      void setup(ros::NodeHandle *nh)
      {
        nh->subscribe(sub_pwm);
        nh->advertise(pub_counter);

        pinMode(pwm_pin, OUTPUT);
        analogWrite(pwm_pin, 0);
  	
        pinMode(encoder_pin, INPUT);
        if(encoder_pin == 2) {	
          attachInterrupt(0, ISR_0, RISING);
        }

        if(encoder_pin == 3) {
          attachInterrupt(1, ISR_1, RISING);
        }

        if(dir_pin >= 0) {
          pinMode(dir_pin, OUTPUT);
	  digitalWrite(dir_pin, LOW);
        }

        if(brake_pin >= 0) {
          pinMode(brake_pin, OUTPUT);
	  digitalWrite(brake_pin, LOW);
        }
      }

        // Max motor speed : 10 000 TR/MIN
        // Min motor speed : 500 TR/MIN (1 REVOLUTION IN ABOUT 10 SECONDS)
        // Encoder pulse by revolution : 4
        // Max pulse / s : 10000 * 4 / 60 = 600 Hz -> 1,6 ms
        // Min pulse / s : 500 * 4 / 60 = 33 Hz -> 30,3 ms
        // Measure timeout 16 ms (half period) -> Usefull when motor is stopped
      void update() {
        // wheel motor use interrupts that modify counter
	// launcher motor do not use interrupt
	// so there is two methods to compute counter
	if(encoder_pin < 4) {
	  counter.data = encoder_counter[encoder_pin - 2];
	} else {
          long measure_in_us = pulseIn(encoder_pin, HIGH, 10000UL);
	  counter.data = (int) round(1e6 / float(measure_in_us)); // Hz
	}
        pub_counter.publish(&counter);
      }

      void updatePWM(const std_msgs::Float64& msg) {
        pwm = (int) round(msg.data);
        digitalWrite(dir_pin, pwm > 0);
        digitalWrite(brake_pin, pwm < 0);
        analogWrite(pwm_pin,abs(pwm) > 255 ? 255 : abs(pwm));
      }
  };
}
#endif
