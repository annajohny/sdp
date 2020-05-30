// Generated by gencpp from file tug_resource_monitor/NodesInfoRequest.msg
// DO NOT EDIT!


#ifndef TUG_RESOURCE_MONITOR_MESSAGE_NODESINFOREQUEST_H
#define TUG_RESOURCE_MONITOR_MESSAGE_NODESINFOREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tug_resource_monitor
{
template <class ContainerAllocator>
struct NodesInfoRequest_
{
  typedef NodesInfoRequest_<ContainerAllocator> Type;

  NodesInfoRequest_()
    : node_names()  {
    }
  NodesInfoRequest_(const ContainerAllocator& _alloc)
    : node_names(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _node_names_type;
  _node_names_type node_names;





  typedef boost::shared_ptr< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> const> ConstPtr;

}; // struct NodesInfoRequest_

typedef ::tug_resource_monitor::NodesInfoRequest_<std::allocator<void> > NodesInfoRequest;

typedef boost::shared_ptr< ::tug_resource_monitor::NodesInfoRequest > NodesInfoRequestPtr;
typedef boost::shared_ptr< ::tug_resource_monitor::NodesInfoRequest const> NodesInfoRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace tug_resource_monitor

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'tug_resource_monitor': ['/home/anna/catkin_ws/src/model_based_diagnosis/tug_resource_monitor/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "42c13fd17030d24481c0409739ac5ae7";
  }

  static const char* value(const ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x42c13fd17030d244ULL;
  static const uint64_t static_value2 = 0x81c0409739ac5ae7ULL;
};

template<class ContainerAllocator>
struct DataType< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tug_resource_monitor/NodesInfoRequest";
  }

  static const char* value(const ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] node_names\n\
";
  }

  static const char* value(const ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.node_names);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NodesInfoRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tug_resource_monitor::NodesInfoRequest_<ContainerAllocator>& v)
  {
    s << indent << "node_names[]" << std::endl;
    for (size_t i = 0; i < v.node_names.size(); ++i)
    {
      s << indent << "  node_names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.node_names[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TUG_RESOURCE_MONITOR_MESSAGE_NODESINFOREQUEST_H