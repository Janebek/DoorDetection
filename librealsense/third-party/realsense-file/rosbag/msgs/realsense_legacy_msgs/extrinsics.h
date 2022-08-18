// Generated by gencpp from file realsense_legacy_msgs/extrinsics.msg
// DO NOT EDIT!


#ifndef realsense_legacy_msgs_MESSAGE_EXTRINSICS_H
#define realsense_legacy_msgs_MESSAGE_EXTRINSICS_H


#include <string>
#include <vector>
#include <array>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace realsense_legacy_msgs
{
template <class ContainerAllocator>
struct extrinsics_
{
  typedef extrinsics_<ContainerAllocator> Type;

  extrinsics_()
    : rotation()
    , translation()  {
      rotation.fill(0.0);

      translation.fill(0.0);
  }
  extrinsics_(const ContainerAllocator& _alloc)
    : rotation()
    , translation()  {
  (void)_alloc;
      rotation.fill(0.0);

      translation.fill(0.0);
  }



   typedef std::array<float, 9>  _rotation_type;
  _rotation_type rotation;

   typedef std::array<float, 3>  _translation_type;
  _translation_type translation;




  typedef std::shared_ptr< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> > Ptr;
  typedef std::shared_ptr< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> const> ConstPtr;

}; // struct extrinsics_

typedef ::realsense_legacy_msgs::extrinsics_<std::allocator<void> > extrinsics;

typedef std::shared_ptr< ::realsense_legacy_msgs::extrinsics > extrinsicsPtr;
typedef std::shared_ptr< ::realsense_legacy_msgs::extrinsics const> extrinsicsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> & v)
{
rs2rosinternal::message_operations::Printer< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace realsense_legacy_msgs

namespace rs2rosinternal
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'realsense_legacy_msgs': ['/home/administrator/realsense_ros_file/realsense_file/realsense_legacy_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "21af2234bc223eca7ed86d7046906de5";
  }

  static const char* value(const ::realsense_legacy_msgs::extrinsics_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x21af2234bc223ecaULL;
  static const uint64_t static_value2 = 0x7ed86d7046906de5ULL;
};

template<class ContainerAllocator>
struct DataType< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "realsense_legacy_msgs/extrinsics";
  }

  static const char* value(const ::realsense_legacy_msgs::extrinsics_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[9] rotation    # column-major 3x3 rotation matrix \n\
float32[3] translation # 3 element translation vector, in meters \n\
";
  }

  static const char* value(const ::realsense_legacy_msgs::extrinsics_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace rs2rosinternal

namespace rs2rosinternal
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.rotation);
      stream.next(m.translation);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct extrinsics_

} // namespace serialization
} // namespace rs2rosinternal

namespace rs2rosinternal
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::realsense_legacy_msgs::extrinsics_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::realsense_legacy_msgs::extrinsics_<ContainerAllocator>& v)
  {
    s << indent << "rotation[]" << std::endl;
    for (size_t i = 0; i < v.rotation.size(); ++i)
    {
      s << indent << "  rotation[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.rotation[i]);
    }
    s << indent << "translation[]" << std::endl;
    for (size_t i = 0; i < v.translation.size(); ++i)
    {
      s << indent << "  translation[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.translation[i]);
    }
  }
};

} // namespace message_operations
} // namespace rs2rosinternal

#endif // realsense_legacy_msgs_MESSAGE_EXTRINSICS_H
