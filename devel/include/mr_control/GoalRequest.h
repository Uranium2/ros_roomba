// Generated by gencpp from file mr_control/GoalRequest.msg
// DO NOT EDIT!


#ifndef MR_CONTROL_MESSAGE_GOALREQUEST_H
#define MR_CONTROL_MESSAGE_GOALREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mr_control
{
template <class ContainerAllocator>
struct GoalRequest_
{
  typedef GoalRequest_<ContainerAllocator> Type;

  GoalRequest_()
    {
    }
  GoalRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::mr_control::GoalRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mr_control::GoalRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GoalRequest_

typedef ::mr_control::GoalRequest_<std::allocator<void> > GoalRequest;

typedef boost::shared_ptr< ::mr_control::GoalRequest > GoalRequestPtr;
typedef boost::shared_ptr< ::mr_control::GoalRequest const> GoalRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mr_control::GoalRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mr_control::GoalRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace mr_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mr_control::GoalRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mr_control::GoalRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mr_control::GoalRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mr_control::GoalRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mr_control::GoalRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mr_control::GoalRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mr_control::GoalRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::mr_control::GoalRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::mr_control::GoalRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mr_control/GoalRequest";
  }

  static const char* value(const ::mr_control::GoalRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mr_control::GoalRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::mr_control::GoalRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mr_control::GoalRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GoalRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mr_control::GoalRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::mr_control::GoalRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // MR_CONTROL_MESSAGE_GOALREQUEST_H
