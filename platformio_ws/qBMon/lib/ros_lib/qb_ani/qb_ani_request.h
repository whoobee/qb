#ifndef _ROS_qb_ani_qb_ani_request_h
#define _ROS_qb_ani_qb_ani_request_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace qb_ani
{

  class qb_ani_request : public ros::Msg
  {
    public:
      typedef uint32_t _request_id_type;
      _request_id_type request_id;
      float rotation_angle[3];
      float rotation_speed[3];
      typedef float _recess_travel_type;
      _recess_travel_type recess_travel;
      typedef float _recess_speed_type;
      _recess_speed_type recess_speed;

    qb_ani_request():
      request_id(0),
      rotation_angle(),
      rotation_speed(),
      recess_travel(0),
      recess_speed(0)
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
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_rotation_anglei;
      u_rotation_anglei.real = this->rotation_angle[i];
      *(outbuffer + offset + 0) = (u_rotation_anglei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rotation_anglei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rotation_anglei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rotation_anglei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rotation_angle[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_rotation_speedi;
      u_rotation_speedi.real = this->rotation_speed[i];
      *(outbuffer + offset + 0) = (u_rotation_speedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rotation_speedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rotation_speedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rotation_speedi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rotation_speed[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_recess_travel;
      u_recess_travel.real = this->recess_travel;
      *(outbuffer + offset + 0) = (u_recess_travel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_recess_travel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_recess_travel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_recess_travel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->recess_travel);
      union {
        float real;
        uint32_t base;
      } u_recess_speed;
      u_recess_speed.real = this->recess_speed;
      *(outbuffer + offset + 0) = (u_recess_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_recess_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_recess_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_recess_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->recess_speed);
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
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_rotation_anglei;
      u_rotation_anglei.base = 0;
      u_rotation_anglei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rotation_anglei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rotation_anglei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rotation_anglei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rotation_angle[i] = u_rotation_anglei.real;
      offset += sizeof(this->rotation_angle[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_rotation_speedi;
      u_rotation_speedi.base = 0;
      u_rotation_speedi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rotation_speedi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rotation_speedi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rotation_speedi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rotation_speed[i] = u_rotation_speedi.real;
      offset += sizeof(this->rotation_speed[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_recess_travel;
      u_recess_travel.base = 0;
      u_recess_travel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_recess_travel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_recess_travel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_recess_travel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->recess_travel = u_recess_travel.real;
      offset += sizeof(this->recess_travel);
      union {
        float real;
        uint32_t base;
      } u_recess_speed;
      u_recess_speed.base = 0;
      u_recess_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_recess_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_recess_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_recess_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->recess_speed = u_recess_speed.real;
      offset += sizeof(this->recess_speed);
     return offset;
    }

    const char * getType(){ return "qb_ani/qb_ani_request"; };
    const char * getMD5(){ return "87b8e830105567a2277945d1ca93de6d"; };

  };

}
#endif
