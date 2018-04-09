// Generated by gencpp from file car_detect/TrackedObject.msg
// DO NOT EDIT!


#ifndef CAR_DETECT_MESSAGE_TRACKEDOBJECT_H
#define CAR_DETECT_MESSAGE_TRACKEDOBJECT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <car_detect/DimensionsWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

namespace car_detect
{
template <class ContainerAllocator>
struct TrackedObject_
{
  typedef TrackedObject_<ContainerAllocator> Type;

  TrackedObject_()
    : header()
    , track_id(0)
    , pose()
    , dims()
    , twist()  {
    }
  TrackedObject_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , track_id(0)
    , pose(_alloc)
    , dims(_alloc)
    , twist(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint64_t _track_id_type;
  _track_id_type track_id;

   typedef  ::geometry_msgs::PoseWithCovariance_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::car_detect::DimensionsWithCovariance_<ContainerAllocator>  _dims_type;
  _dims_type dims;

   typedef  ::geometry_msgs::TwistWithCovariance_<ContainerAllocator>  _twist_type;
  _twist_type twist;




  typedef boost::shared_ptr< ::car_detect::TrackedObject_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::car_detect::TrackedObject_<ContainerAllocator> const> ConstPtr;

}; // struct TrackedObject_

typedef ::car_detect::TrackedObject_<std::allocator<void> > TrackedObject;

typedef boost::shared_ptr< ::car_detect::TrackedObject > TrackedObjectPtr;
typedef boost::shared_ptr< ::car_detect::TrackedObject const> TrackedObjectConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::car_detect::TrackedObject_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::car_detect::TrackedObject_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace car_detect

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/lunar/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/lunar/share/std_msgs/cmake/../msg'], 'car_detect': ['/home/demikandr/SideProjects/car-detect/catkin_ws/src/car_detect/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::car_detect::TrackedObject_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::car_detect::TrackedObject_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::car_detect::TrackedObject_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::car_detect::TrackedObject_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::car_detect::TrackedObject_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::car_detect::TrackedObject_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::car_detect::TrackedObject_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bfc1e1864ae13bc7372710257629bec7";
  }

  static const char* value(const ::car_detect::TrackedObject_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbfc1e1864ae13bc7ULL;
  static const uint64_t static_value2 = 0x372710257629bec7ULL;
};

template<class ContainerAllocator>
struct DataType< ::car_detect::TrackedObject_<ContainerAllocator> >
{
  static const char* value()
  {
    return "car_detect/TrackedObject";
  }

  static const char* value(const ::car_detect::TrackedObject_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::car_detect::TrackedObject_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header              header\n\
uint64        track_id\n\
# pose: position and orientation\n\
geometry_msgs/PoseWithCovariance        pose\n\
# dimensions of the object\n\
DimensionsWithCovariance        dims\n\
# velocity linear and angular\n\
geometry_msgs/TwistWithCovariance twist\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: car_detect/DimensionsWithCovariance\n\
geometry_msgs/Vector3            dimensions     # sizes of the bounding box\n\
float64[9] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/TwistWithCovariance\n\
# This expresses velocity in free space with uncertainty.\n\
\n\
Twist twist\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
";
  }

  static const char* value(const ::car_detect::TrackedObject_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::car_detect::TrackedObject_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.track_id);
      stream.next(m.pose);
      stream.next(m.dims);
      stream.next(m.twist);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrackedObject_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::car_detect::TrackedObject_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::car_detect::TrackedObject_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "track_id: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.track_id);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "dims: ";
    s << std::endl;
    Printer< ::car_detect::DimensionsWithCovariance_<ContainerAllocator> >::stream(s, indent + "  ", v.dims);
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::geometry_msgs::TwistWithCovariance_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAR_DETECT_MESSAGE_TRACKEDOBJECT_H
