#ifndef _STARDUST_ASC712_h
#define _STARDUST_ASC712_h
#include <std_msgs/Int16.h>
#include <math.h>

namespace stardust
{
  class ACS712
  {
    private:
    
      const byte pin;
      std_msgs::Int16 current;
      ros::Publisher pub_current;

    public :
    
      ACS712(const char *_topic_name, const byte _pin):
          pin(_pin),
          pub_current(_topic_name, &current) { }
      
      void setup(ros::NodeHandle *nh) {
        nh->advertise(pub_current);
        pinMode(pin, INPUT);
      }

      // Current in mA
      // ACS512 5A specs : 512=>0mA, 1024=>5000mA, 0=>-5000mA
      // Resolution : 9,77 mA for 1 bit
      void measureAndPublish() {
        float c = 0;
	for(int i=0;i<4;i++) {
	  c += (float) (analogRead(pin) - 512);
	}	
        current.data = (int) round(-2.44 * c);
        pub_current.publish(&current);
      }
  };
}
#endif
