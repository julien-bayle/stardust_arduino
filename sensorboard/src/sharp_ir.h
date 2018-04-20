#ifndef _STARDUST_SHARP_GP2Y0A51SK0F_h
#define _STARDUST_SHARP_GP2Y0A51SK0F_h
#include <sensor_msgs/Range.h>

namespace stardust
{
  class SHARP_GP2Y0A51SK0F 
  {
    private:
    
      const byte pin;
      
      sensor_msgs::Range range_msg;
      ros::Publisher pub_range;

    public :

      SHARP_GP2Y0A51SK0F(const char *_topic_name,
        const byte _pin):
          pin(_pin),
          pub_range(_topic_name, &range_msg) {
            range_msg.radiation_type = sensor_msgs::Range::INFRARED;
            range_msg.header.frame_id =  _topic_name;
            range_msg.field_of_view = 0.01; // rad (0.5 degree)
            range_msg.min_range = 0.02;     // meter (2 cm)
            range_msg.max_range = 0.15;     // meter (15 cm)
      }
    
      void setup(ros::NodeHandle *nh)
      {
        nh->advertise(pub_range);
      }

      /*
       * getRange() - samples the analog input from the ranger
       * and converts it into meters.  
       * 
       * Voltage at max distance is 0.4 V 
       * So if measure is below 0.4/(5/1024) = 82, 
       * we cannot determine the real distance.
       * In this case, max distance is returned () 
       *
       * Linearisation with https://mycurvefit.com/
       * Fit method :
       * distance = a/(measure+b) 
       * 
       * Data :
       * 2.10 V => 430 => 0.02 m
       * 1.05 V => 215 => 0.05 m
       * 0.60V => 123 => 0.10 m
       * 0.40 V => 82 => 0.15 m 
       *
       * Result :
       * distance = 10.48688/(measure-12.80125)
       */
      float getRange(){
        int measure = analogRead(pin);
        if(measure < 82)
          return 254;

        return 10.48688/(measure - 12.80125);
      }
  
      void measureAndPublish(ros::NodeHandle *nh) {
        range_msg.range = getRange();
        range_msg.header.stamp = nh->now();
        pub_range.publish(&range_msg);
      }
  };
}
#endif
