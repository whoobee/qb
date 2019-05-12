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
      typedef int16_t _speed_l_type;
      _speed_l_type speed_l;
      typedef int16_t _speed_r_type;
      _speed_r_type speed_r;

    wheel_telemetry():
      speed_l(0),
      speed_r(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_speed_l;
      u_speed_l.real = this->speed_l;
      *(outbuffer + offset + 0) = (u_speed_l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_l.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed_l);
      union {
        int16_t real;
        uint16_t base;
      } u_speed_r;
      u_speed_r.real = this->speed_r;
      *(outbuffer + offset + 0) = (u_speed_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_r.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed_r);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_speed_l;
      u_speed_l.base = 0;
      u_speed_l.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_l.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed_l = u_speed_l.real;
      offset += sizeof(this->speed_l);
      union {
        int16_t real;
        uint16_t base;
      } u_speed_r;
      u_speed_r.base = 0;
      u_speed_r.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_r.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed_r = u_speed_r.real;
      offset += sizeof(this->speed_r);
     return offset;
    }

    const char * getType(){ return "mirobot_driver/wheel_telemetry"; };
    const char * getMD5(){ return "6a13c855fd502a6ed208724a11e8020c"; };

  };

}
#endif
