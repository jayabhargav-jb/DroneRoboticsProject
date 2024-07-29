// Generated by gencpp from file diffbot_msgs/PID.msg
// DO NOT EDIT!


#ifndef DIFFBOT_MSGS_MESSAGE_PID_H
#define DIFFBOT_MSGS_MESSAGE_PID_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace diffbot_msgs
{
template <class ContainerAllocator>
struct PID_
{
  typedef PID_<ContainerAllocator> Type;

  PID_()
    : kp(0.0)
    , ki(0.0)
    , kd(0.0)  {
    }
  PID_(const ContainerAllocator& _alloc)
    : kp(0.0)
    , ki(0.0)
    , kd(0.0)  {
  (void)_alloc;
    }



   typedef float _kp_type;
  _kp_type kp;

   typedef float _ki_type;
  _ki_type ki;

   typedef float _kd_type;
  _kd_type kd;





  typedef boost::shared_ptr< ::diffbot_msgs::PID_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::diffbot_msgs::PID_<ContainerAllocator> const> ConstPtr;

}; // struct PID_

typedef ::diffbot_msgs::PID_<std::allocator<void> > PID;

typedef boost::shared_ptr< ::diffbot_msgs::PID > PIDPtr;
typedef boost::shared_ptr< ::diffbot_msgs::PID const> PIDConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::diffbot_msgs::PID_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::diffbot_msgs::PID_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::diffbot_msgs::PID_<ContainerAllocator1> & lhs, const ::diffbot_msgs::PID_<ContainerAllocator2> & rhs)
{
  return lhs.kp == rhs.kp &&
    lhs.ki == rhs.ki &&
    lhs.kd == rhs.kd;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::diffbot_msgs::PID_<ContainerAllocator1> & lhs, const ::diffbot_msgs::PID_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace diffbot_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::diffbot_msgs::PID_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::diffbot_msgs::PID_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::diffbot_msgs::PID_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::diffbot_msgs::PID_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::diffbot_msgs::PID_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::diffbot_msgs::PID_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::diffbot_msgs::PID_<ContainerAllocator> >
{
  static const char* value()
  {
    return "08d0ca1f582560895ecba909de1d88ec";
  }

  static const char* value(const ::diffbot_msgs::PID_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x08d0ca1f58256089ULL;
  static const uint64_t static_value2 = 0x5ecba909de1d88ecULL;
};

template<class ContainerAllocator>
struct DataType< ::diffbot_msgs::PID_<ContainerAllocator> >
{
  static const char* value()
  {
    return "diffbot_msgs/PID";
  }

  static const char* value(const ::diffbot_msgs::PID_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::diffbot_msgs::PID_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This is a message to hold the PID tuning values\n"
"# Required only for the low level rosserial base_controller\n"
"# because rosserial doesn't support dynamic reconfigure\n"
"float32 kp\n"
"float32 ki\n"
"float32 kd\n"
;
  }

  static const char* value(const ::diffbot_msgs::PID_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::diffbot_msgs::PID_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.kp);
      stream.next(m.ki);
      stream.next(m.kd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PID_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::diffbot_msgs::PID_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::diffbot_msgs::PID_<ContainerAllocator>& v)
  {
    s << indent << "kp: ";
    Printer<float>::stream(s, indent + "  ", v.kp);
    s << indent << "ki: ";
    Printer<float>::stream(s, indent + "  ", v.ki);
    s << indent << "kd: ";
    Printer<float>::stream(s, indent + "  ", v.kd);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DIFFBOT_MSGS_MESSAGE_PID_H
