#ifndef _ROS_qb_ani_qb_ani_status_h
#define _ROS_qb_ani_qb_ani_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace qb_ani
{

  class qb_ani_status : public ros::Msg
  {
    public:
      typedef uint32_t _request_id_type;
      _request_id_type request_id;
      typedef uint8_t _status_type;
      _status_type status;
      enum { ANI_STATUS_IDLE = 0 };
      enum { ANI_STATUS_RUNNIG = 1 };
      enum { ANI_STATUS_UNKNOWN_POSITION = 254 };
      enum { ANI_STATUS_GENERIC_ERROR = 255 };

    qb_ani_status():
      request_id(0),
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->request_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->request_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->request_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->request_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->request_id);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->request_id =  ((uint32_t) (*(inbuffer + offset)));
      this->request_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->request_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->request_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->request_id);
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return "qb_ani/qb_ani_status"; };
    const char * getMD5(){ return "4f05a6f1d7544deb57bdb1e691a30af7"; };

  };

}
#endif
