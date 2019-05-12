// Generated by gencpp from file mirobot_driver/bot_telemetry.msg
// DO NOT EDIT!


#ifndef MIROBOT_DRIVER_MESSAGE_BOT_TELEMETRY_H
#define MIROBOT_DRIVER_MESSAGE_BOT_TELEMETRY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mirobot_driver
{
template <class ContainerAllocator>
struct bot_telemetry_
{
  typedef bot_telemetry_<ContainerAllocator> Type;

  bot_telemetry_()
    : id(0)
    , gyro_x(0)
    , gyro_y(0)
    , gyro_z(0)
    , acc_x(0)
    , acc_y(0)
    , acc_z(0)  {
    }
  bot_telemetry_(const ContainerAllocator& _alloc)
    : id(0)
    , gyro_x(0)
    , gyro_y(0)
    , gyro_z(0)
    , acc_x(0)
    , acc_y(0)
    , acc_z(0)  {
  (void)_alloc;
    }



   typedef uint8_t _id_type;
  _id_type id;

   typedef int32_t _gyro_x_type;
  _gyro_x_type gyro_x;

   typedef int32_t _gyro_y_type;
  _gyro_y_type gyro_y;

   typedef int32_t _gyro_z_type;
  _gyro_z_type gyro_z;

   typedef int32_t _acc_x_type;
  _acc_x_type acc_x;

   typedef int32_t _acc_y_type;
  _acc_y_type acc_y;

   typedef int32_t _acc_z_type;
  _acc_z_type acc_z;





  typedef boost::shared_ptr< ::mirobot_driver::bot_telemetry_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mirobot_driver::bot_telemetry_<ContainerAllocator> const> ConstPtr;

}; // struct bot_telemetry_

typedef ::mirobot_driver::bot_telemetry_<std::allocator<void> > bot_telemetry;

typedef boost::shared_ptr< ::mirobot_driver::bot_telemetry > bot_telemetryPtr;
typedef boost::shared_ptr< ::mirobot_driver::bot_telemetry const> bot_telemetryConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mirobot_driver::bot_telemetry_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mirobot_driver::bot_telemetry_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mirobot_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'mirobot_driver': ['/home/whoobee/mirobot/ros_ws/src/mirobot_driver/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mirobot_driver::bot_telemetry_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mirobot_driver::bot_telemetry_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mirobot_driver::bot_telemetry_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mirobot_driver::bot_telemetry_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mirobot_driver::bot_telemetry_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mirobot_driver::bot_telemetry_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mirobot_driver::bot_telemetry_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c924c03de4741fb4b40f30ce9f8a6fe4";
  }

  static const char* value(const ::mirobot_driver::bot_telemetry_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc924c03de4741fb4ULL;
  static const uint64_t static_value2 = 0xb40f30ce9f8a6fe4ULL;
};

template<class ContainerAllocator>
struct DataType< ::mirobot_driver::bot_telemetry_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mirobot_driver/bot_telemetry";
  }

  static const char* value(const ::mirobot_driver::bot_telemetry_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mirobot_driver::bot_telemetry_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 id\n"
"int32 gyro_x\n"
"int32 gyro_y\n"
"int32 gyro_z\n"
"int32 acc_x\n"
"int32 acc_y\n"
"int32 acc_z\n"
;
  }

  static const char* value(const ::mirobot_driver::bot_telemetry_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mirobot_driver::bot_telemetry_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.gyro_x);
      stream.next(m.gyro_y);
      stream.next(m.gyro_z);
      stream.next(m.acc_x);
      stream.next(m.acc_y);
      stream.next(m.acc_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct bot_telemetry_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mirobot_driver::bot_telemetry_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mirobot_driver::bot_telemetry_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id);
    s << indent << "gyro_x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gyro_x);
    s << indent << "gyro_y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gyro_y);
    s << indent << "gyro_z: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gyro_z);
    s << indent << "acc_x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.acc_x);
    s << indent << "acc_y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.acc_y);
    s << indent << "acc_z: ";
    Printer<int32_t>::stream(s, indent + "  ", v.acc_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MIROBOT_DRIVER_MESSAGE_BOT_TELEMETRY_H
