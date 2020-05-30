// Generated by gencpp from file tug_diagnosis_msgs/observer_configuration.msg
// DO NOT EDIT!


#ifndef TUG_DIAGNOSIS_MSGS_MESSAGE_OBSERVER_CONFIGURATION_H
#define TUG_DIAGNOSIS_MSGS_MESSAGE_OBSERVER_CONFIGURATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tug_diagnosis_msgs
{
template <class ContainerAllocator>
struct observer_configuration_
{
  typedef observer_configuration_<ContainerAllocator> Type;

  observer_configuration_()
    : type()
    , resource()  {
    }
  observer_configuration_(const ContainerAllocator& _alloc)
    : type(_alloc)
    , resource(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  _type_type type;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _resource_type;
  _resource_type resource;





  typedef boost::shared_ptr< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> const> ConstPtr;

}; // struct observer_configuration_

typedef ::tug_diagnosis_msgs::observer_configuration_<std::allocator<void> > observer_configuration;

typedef boost::shared_ptr< ::tug_diagnosis_msgs::observer_configuration > observer_configurationPtr;
typedef boost::shared_ptr< ::tug_diagnosis_msgs::observer_configuration const> observer_configurationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace tug_diagnosis_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'tug_diagnosis_msgs': ['/home/anna/catkin_ws/src/model_based_diagnosis/tug_diagnosis/tug_diagnosis_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a2f5ebb423c16ad934f5d2cae333e2df";
  }

  static const char* value(const ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa2f5ebb423c16ad9ULL;
  static const uint64_t static_value2 = 0x34f5d2cae333e2dfULL;
};

template<class ContainerAllocator>
struct DataType< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tug_diagnosis_msgs/observer_configuration";
  }

  static const char* value(const ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string type\n\
string[] resource\n\
";
  }

  static const char* value(const ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.resource);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct observer_configuration_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tug_diagnosis_msgs::observer_configuration_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
    s << indent << "resource[]" << std::endl;
    for (size_t i = 0; i < v.resource.size(); ++i)
    {
      s << indent << "  resource[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.resource[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TUG_DIAGNOSIS_MSGS_MESSAGE_OBSERVER_CONFIGURATION_H