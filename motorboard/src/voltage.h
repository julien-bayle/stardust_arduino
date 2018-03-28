#ifndef _STARDUST_VOLTAGE_h
#define _STARDUST_VOLTAGE_h
#include <std_msgs/Float32.h>

namespace stardust
{
  class Voltage
  {
    private:
    
      const byte pin;
      const float diviser;
      
      std_msgs::Float32 voltage;
      ros::Publisher pub_voltage;

    public :

      Voltage(const char *_topic_name,
        const byte _pin,
        const float _diviser):
          pin(_pin),
	  diviser(_diviser),
          pub_voltage(_topic_name, &voltage) {}
    
      void setup(ros::NodeHandle *nh)
      {
        nh->advertise(pub_voltage);
        pinMode(pin, INPUT);
      }
  
      void measureAndPublish() {
        int v = analogRead(pin);
        voltage.data = diviser * 5 * ((float) v)/1024;
        pub_voltage.publish(&voltage);
      }
  };
}
#endif
