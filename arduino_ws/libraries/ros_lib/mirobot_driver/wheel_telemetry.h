#ifndef _ROS_mirobot_driver_wheel_telemetry_h
#define _ROS_mirobot_driver_wheel_telemetry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mirobot_driver
{

  class wheel_telemetry : public ros::Msg
  {
    public:
      typedef uint8_t _id_type;
      _id_type id;
      typedef int16_t _speed_type;
      _speed_type speed;

    wheel_telemetry():
      id(0),
      speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int16_t real;
        uint16_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      union {
        int16_t real;
        uint16_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
     return offset;
    }

    const char * getType(){ return "mirobot_driver/wheel_telemetry"; };
    const char * getMD5(){ return "c3660f6f95dba97d26a551ef14e85564"; };

  };

}
#endif
