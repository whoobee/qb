#ifndef _ROS_mirobot_driver_wheel_control_h
#define _ROS_mirobot_driver_wheel_control_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mirobot_driver
{

  class wheel_control : public ros::Msg
  {
    public:
      typedef uint8_t _dir_l_type;
      _dir_l_type dir_l;
      typedef uint8_t _speed_l_type;
      _speed_l_type speed_l;
      typedef uint8_t _dir_r_type;
      _dir_r_type dir_r;
      typedef uint8_t _speed_r_type;
      _speed_r_type speed_r;

    wheel_control():
      dir_l(0),
      speed_l(0),
      dir_r(0),
      speed_r(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->dir_l >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dir_l);
      *(outbuffer + offset + 0) = (this->speed_l >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed_l);
      *(outbuffer + offset + 0) = (this->dir_r >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dir_r);
      *(outbuffer + offset + 0) = (this->speed_r >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed_r);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->dir_l =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dir_l);
      this->speed_l =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->speed_l);
      this->dir_r =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dir_r);
      this->speed_r =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->speed_r);
     return offset;
    }

    const char * getType(){ return "mirobot_driver/wheel_control"; };
    const char * getMD5(){ return "7bf657bb437a3a998dcf9c0bf0cd51fa"; };

  };

}
#endif
