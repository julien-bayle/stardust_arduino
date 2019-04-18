#ifndef _STARDUST_ILS_h
#define _STARDUST_ILS_h
#include <std_msgs/Bool.h>

namespace stardust
{
  class ILS 
  {
    private:
    
      const byte pin;
      
      std_msgs::Bool ils_msg;
      ros::Publisher pub_ils;

    public :

      ILS(const char *_topic_name,
        const byte _pin):
          pin(_pin),
          pub_ils(_topic_name, &ils_msg) {
      }
    
      void setup(ros::NodeHandle *nh)
      {
        nh->advertise(pub_ils);
      }

      void measureAndPublish(ros::NodeHandle *nh) {
	ils_msg.data = analogRead(pin) > 100;
        pub_ils.publish(&ils_msg);
      }
  };
}
#endif
