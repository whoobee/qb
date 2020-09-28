#ifndef _ROS_qb_mon_qb_sys_status_h
#define _ROS_qb_mon_qb_sys_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace qb_mon
{

  class qb_sys_status : public ros::Msg
  {
    public:
      typedef uint8_t _status_type;
      _status_type status;
      enum { STATUS_OFF = 0 };
      enum { STATUS_ON = 1 };
      enum { STATUS_INIT = 2 };
      enum { STATUS_READY = 3 };
      enum { STATUS_RUNNING = 4 };
      enum { STATUS_CHARGING = 5 };
      enum { STATUS_ERROR = 6 };

    qb_sys_status():
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return "qb_mon/qb_sys_status"; };
    const char * getMD5(){ return "b22ab9f56ddd49d401052454bf7a64a8"; };

  };

}
#endif
