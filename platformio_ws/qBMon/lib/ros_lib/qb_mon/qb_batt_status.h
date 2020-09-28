#ifndef _ROS_qb_mon_qb_batt_status_h
#define _ROS_qb_mon_qb_batt_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace qb_mon
{

  class qb_batt_status : public ros::Msg
  {
    public:
      typedef int32_t _id_type;
      _id_type id;
      typedef uint8_t _status_type;
      _status_type status;
      typedef uint8_t _type_type;
      _type_type type;
      typedef float _capacity_type;
      _capacity_type capacity;
      typedef float _max_capacity_type;
      _max_capacity_type max_capacity;
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef const char* _serial_number_type;
      _serial_number_type serial_number;
      enum { STATUS_READY = 0 };
      enum { STATUS_CHARGING = 1 };
      enum { STATUS_UNDERVOLTAGE = 2 };
      enum { STATUS_OVERVOLTAGE = 3 };
      enum { STATUS_UNKNOWN_ERROR = 4 };

    qb_batt_status():
      id(0),
      status(0),
      type(0),
      capacity(0),
      max_capacity(0),
      voltage(0),
      serial_number("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      union {
        float real;
        uint32_t base;
      } u_capacity;
      u_capacity.real = this->capacity;
      *(outbuffer + offset + 0) = (u_capacity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_capacity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_capacity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_capacity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->capacity);
      union {
        float real;
        uint32_t base;
      } u_max_capacity;
      u_max_capacity.real = this->max_capacity;
      *(outbuffer + offset + 0) = (u_max_capacity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_capacity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_capacity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_capacity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_capacity);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      uint32_t length_serial_number = strlen(this->serial_number);
      varToArr(outbuffer + offset, length_serial_number);
      offset += 4;
      memcpy(outbuffer + offset, this->serial_number, length_serial_number);
      offset += length_serial_number;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      union {
        float real;
        uint32_t base;
      } u_capacity;
      u_capacity.base = 0;
      u_capacity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_capacity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_capacity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_capacity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->capacity = u_capacity.real;
      offset += sizeof(this->capacity);
      union {
        float real;
        uint32_t base;
      } u_max_capacity;
      u_max_capacity.base = 0;
      u_max_capacity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_capacity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_capacity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_capacity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_capacity = u_max_capacity.real;
      offset += sizeof(this->max_capacity);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
      uint32_t length_serial_number;
      arrToVar(length_serial_number, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_serial_number; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_serial_number-1]=0;
      this->serial_number = (char *)(inbuffer + offset-1);
      offset += length_serial_number;
     return offset;
    }

    const char * getType(){ return "qb_mon/qb_batt_status"; };
    const char * getMD5(){ return "f6aff88d94ae68a4dba2f99104e74145"; };

  };

}
#endif
