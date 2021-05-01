// Generated by gencpp from file opl_msgs/LocationStatusOp.msg
// DO NOT EDIT!


#ifndef OPL_MSGS_MESSAGE_LOCATIONSTATUSOP_H
#define OPL_MSGS_MESSAGE_LOCATIONSTATUSOP_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace opl_msgs
{
template <class ContainerAllocator>
struct LocationStatusOp_
{
  typedef LocationStatusOp_<ContainerAllocator> Type;

  LocationStatusOp_()
    : location()
    , cube_at_ds(false)
    , op_cube_id()  {
    }
  LocationStatusOp_(const ContainerAllocator& _alloc)
    : location(_alloc)
    , cube_at_ds(false)
    , op_cube_id(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _location_type;
  _location_type location;

   typedef uint8_t _cube_at_ds_type;
  _cube_at_ds_type cube_at_ds;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _op_cube_id_type;
  _op_cube_id_type op_cube_id;





  typedef boost::shared_ptr< ::opl_msgs::LocationStatusOp_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::opl_msgs::LocationStatusOp_<ContainerAllocator> const> ConstPtr;

}; // struct LocationStatusOp_

typedef ::opl_msgs::LocationStatusOp_<std::allocator<void> > LocationStatusOp;

typedef boost::shared_ptr< ::opl_msgs::LocationStatusOp > LocationStatusOpPtr;
typedef boost::shared_ptr< ::opl_msgs::LocationStatusOp const> LocationStatusOpConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::opl_msgs::LocationStatusOp_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::opl_msgs::LocationStatusOp_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::opl_msgs::LocationStatusOp_<ContainerAllocator1> & lhs, const ::opl_msgs::LocationStatusOp_<ContainerAllocator2> & rhs)
{
  return lhs.location == rhs.location &&
    lhs.cube_at_ds == rhs.cube_at_ds &&
    lhs.op_cube_id == rhs.op_cube_id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::opl_msgs::LocationStatusOp_<ContainerAllocator1> & lhs, const ::opl_msgs::LocationStatusOp_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace opl_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::opl_msgs::LocationStatusOp_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::opl_msgs::LocationStatusOp_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::opl_msgs::LocationStatusOp_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::opl_msgs::LocationStatusOp_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::opl_msgs::LocationStatusOp_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::opl_msgs::LocationStatusOp_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::opl_msgs::LocationStatusOp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d7e59fda5792b384bfe9ef98e01f915a";
  }

  static const char* value(const ::opl_msgs::LocationStatusOp_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd7e59fda5792b384ULL;
  static const uint64_t static_value2 = 0xbfe9ef98e01f915aULL;
};

template<class ContainerAllocator>
struct DataType< ::opl_msgs::LocationStatusOp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "opl_msgs/LocationStatusOp";
  }

  static const char* value(const ::opl_msgs::LocationStatusOp_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::opl_msgs::LocationStatusOp_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string location\n"
"bool cube_at_ds\n"
"string op_cube_id\n"
;
  }

  static const char* value(const ::opl_msgs::LocationStatusOp_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::opl_msgs::LocationStatusOp_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.location);
      stream.next(m.cube_at_ds);
      stream.next(m.op_cube_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LocationStatusOp_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::opl_msgs::LocationStatusOp_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::opl_msgs::LocationStatusOp_<ContainerAllocator>& v)
  {
    s << indent << "location: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.location);
    s << indent << "cube_at_ds: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cube_at_ds);
    s << indent << "op_cube_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.op_cube_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPL_MSGS_MESSAGE_LOCATIONSTATUSOP_H
