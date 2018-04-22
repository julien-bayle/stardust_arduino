#ifndef _STARDUST_ULTRASOUND_h
#define _STARDUST_ULTRASOUND_h
#include <sensor_msgs/Range.h>
#include <math.h>

namespace stardust
{

  /* HC-SR04 timeout */
  const unsigned long MEASURE_TIMEOUT = 20000UL; // 20ms = ~6.8m at 340m/s

  /* Sound speed in m/us (two roud trip : 340 / 1e6 / 2) */
  const float SOUND_SPEED_2 = 0.00017;

  class SonarHcSr04
  {
    private:
    
      const byte echo_pin;
      const byte trigger_pin;

      sensor_msgs::Range range_msg;
      ros::Publisher pub_range;

    public :

      SonarHcSr04(const char *_topic_name,
        const byte _echo_pin, 
        const byte _trigger_pin):
          echo_pin(_echo_pin),
          trigger_pin(_trigger_pin),
          pub_range(_topic_name, &range_msg) {
            range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
            range_msg.header.frame_id =  _topic_name;
            range_msg.field_of_view = 0.5; // rad (30° degrees)
            range_msg.min_range = 0.08;    // meter (8 cm)
            range_msg.max_range = 0.60;    // meter
      }
    
      void setup(ros::NodeHandle *nh)
      {
        nh->advertise(pub_range);

        pinMode(echo_pin, INPUT);
        pinMode(trigger_pin, OUTPUT);
        analogWrite(trigger_pin, LOW);
      }

      float getRange(){
        /* Trigger an ultrasonic sound for 10us */
        digitalWrite(trigger_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger_pin, LOW);

        /* Mesure echo time */
        long measure = pulseIn(echo_pin, HIGH, MEASURE_TIMEOUT);

        /* Convert time to a distance */
        if (measure > 0)
          return measure * SOUND_SPEED_2;
        else
          return 10.0; // 10 meters (ROS will ignore it as it is greater than max range)
      }
  
      void measureAndPublish(ros::NodeHandle *nh) {
        range_msg.range = getRange();
        range_msg.header.stamp = nh->now();
        pub_range.publish(&range_msg);
      }
  };
}
#endif
