// Generated by gencpp from file quadrotor_msgs/Px4ctrlDebug.msg
// DO NOT EDIT!


#ifndef QUADROTOR_MSGS_MESSAGE_PX4CTRLDEBUG_H
#define QUADROTOR_MSGS_MESSAGE_PX4CTRLDEBUG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct Px4ctrlDebug_
{
  typedef Px4ctrlDebug_<ContainerAllocator> Type;

  Px4ctrlDebug_()
    : header()
    , des_p_x(0.0)
    , des_p_y(0.0)
    , des_p_z(0.0)
    , des_v_x(0.0)
    , des_v_y(0.0)
    , des_v_z(0.0)
    , fb_a_x(0.0)
    , fb_a_y(0.0)
    , fb_a_z(0.0)
    , des_a_x(0.0)
    , des_a_y(0.0)
    , des_a_z(0.0)
    , des_q_x(0.0)
    , des_q_y(0.0)
    , des_q_z(0.0)
    , des_q_w(0.0)
    , des_thr(0.0)
    , thr2acc(0.0)
    , thr_scale_compensate(0.0)
    , voltage(0.0)
    , err_axisang_x(0.0)
    , err_axisang_y(0.0)
    , err_axisang_z(0.0)
    , err_axisang_ang(0.0)
    , fb_rate_x(0.0)
    , fb_rate_y(0.0)
    , fb_rate_z(0.0)  {
    }
  Px4ctrlDebug_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , des_p_x(0.0)
    , des_p_y(0.0)
    , des_p_z(0.0)
    , des_v_x(0.0)
    , des_v_y(0.0)
    , des_v_z(0.0)
    , fb_a_x(0.0)
    , fb_a_y(0.0)
    , fb_a_z(0.0)
    , des_a_x(0.0)
    , des_a_y(0.0)
    , des_a_z(0.0)
    , des_q_x(0.0)
    , des_q_y(0.0)
    , des_q_z(0.0)
    , des_q_w(0.0)
    , des_thr(0.0)
    , thr2acc(0.0)
    , thr_scale_compensate(0.0)
    , voltage(0.0)
    , err_axisang_x(0.0)
    , err_axisang_y(0.0)
    , err_axisang_z(0.0)
    , err_axisang_ang(0.0)
    , fb_rate_x(0.0)
    , fb_rate_y(0.0)
    , fb_rate_z(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _des_p_x_type;
  _des_p_x_type des_p_x;

   typedef double _des_p_y_type;
  _des_p_y_type des_p_y;

   typedef double _des_p_z_type;
  _des_p_z_type des_p_z;

   typedef double _des_v_x_type;
  _des_v_x_type des_v_x;

   typedef double _des_v_y_type;
  _des_v_y_type des_v_y;

   typedef double _des_v_z_type;
  _des_v_z_type des_v_z;

   typedef double _fb_a_x_type;
  _fb_a_x_type fb_a_x;

   typedef double _fb_a_y_type;
  _fb_a_y_type fb_a_y;

   typedef double _fb_a_z_type;
  _fb_a_z_type fb_a_z;

   typedef double _des_a_x_type;
  _des_a_x_type des_a_x;

   typedef double _des_a_y_type;
  _des_a_y_type des_a_y;

   typedef double _des_a_z_type;
  _des_a_z_type des_a_z;

   typedef double _des_q_x_type;
  _des_q_x_type des_q_x;

   typedef double _des_q_y_type;
  _des_q_y_type des_q_y;

   typedef double _des_q_z_type;
  _des_q_z_type des_q_z;

   typedef double _des_q_w_type;
  _des_q_w_type des_q_w;

   typedef double _des_thr_type;
  _des_thr_type des_thr;

   typedef double _thr2acc_type;
  _thr2acc_type thr2acc;

   typedef double _thr_scale_compensate_type;
  _thr_scale_compensate_type thr_scale_compensate;

   typedef double _voltage_type;
  _voltage_type voltage;

   typedef double _err_axisang_x_type;
  _err_axisang_x_type err_axisang_x;

   typedef double _err_axisang_y_type;
  _err_axisang_y_type err_axisang_y;

   typedef double _err_axisang_z_type;
  _err_axisang_z_type err_axisang_z;

   typedef double _err_axisang_ang_type;
  _err_axisang_ang_type err_axisang_ang;

   typedef double _fb_rate_x_type;
  _fb_rate_x_type fb_rate_x;

   typedef double _fb_rate_y_type;
  _fb_rate_y_type fb_rate_y;

   typedef double _fb_rate_z_type;
  _fb_rate_z_type fb_rate_z;





  typedef boost::shared_ptr< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> const> ConstPtr;

}; // struct Px4ctrlDebug_

typedef ::quadrotor_msgs::Px4ctrlDebug_<std::allocator<void> > Px4ctrlDebug;

typedef boost::shared_ptr< ::quadrotor_msgs::Px4ctrlDebug > Px4ctrlDebugPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::Px4ctrlDebug const> Px4ctrlDebugConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator1> & lhs, const ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.des_p_x == rhs.des_p_x &&
    lhs.des_p_y == rhs.des_p_y &&
    lhs.des_p_z == rhs.des_p_z &&
    lhs.des_v_x == rhs.des_v_x &&
    lhs.des_v_y == rhs.des_v_y &&
    lhs.des_v_z == rhs.des_v_z &&
    lhs.fb_a_x == rhs.fb_a_x &&
    lhs.fb_a_y == rhs.fb_a_y &&
    lhs.fb_a_z == rhs.fb_a_z &&
    lhs.des_a_x == rhs.des_a_x &&
    lhs.des_a_y == rhs.des_a_y &&
    lhs.des_a_z == rhs.des_a_z &&
    lhs.des_q_x == rhs.des_q_x &&
    lhs.des_q_y == rhs.des_q_y &&
    lhs.des_q_z == rhs.des_q_z &&
    lhs.des_q_w == rhs.des_q_w &&
    lhs.des_thr == rhs.des_thr &&
    lhs.thr2acc == rhs.thr2acc &&
    lhs.thr_scale_compensate == rhs.thr_scale_compensate &&
    lhs.voltage == rhs.voltage &&
    lhs.err_axisang_x == rhs.err_axisang_x &&
    lhs.err_axisang_y == rhs.err_axisang_y &&
    lhs.err_axisang_z == rhs.err_axisang_z &&
    lhs.err_axisang_ang == rhs.err_axisang_ang &&
    lhs.fb_rate_x == rhs.fb_rate_x &&
    lhs.fb_rate_y == rhs.fb_rate_y &&
    lhs.fb_rate_z == rhs.fb_rate_z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator1> & lhs, const ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9ddf78a965614929f792cafb89e47f73";
  }

  static const char* value(const ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9ddf78a965614929ULL;
  static const uint64_t static_value2 = 0xf792cafb89e47f73ULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> >
{
  static const char* value()
  {
    return "quadrotor_msgs/Px4ctrlDebug";
  }

  static const char* value(const ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"float64 des_p_x\n"
"float64 des_p_y\n"
"float64 des_p_z\n"
"\n"
"float64 des_v_x\n"
"float64 des_v_y\n"
"float64 des_v_z\n"
"\n"
"float64 fb_a_x\n"
"float64 fb_a_y\n"
"float64 fb_a_z\n"
"\n"
"float64 des_a_x\n"
"float64 des_a_y\n"
"float64 des_a_z\n"
"\n"
"float64 des_q_x\n"
"float64 des_q_y\n"
"float64 des_q_z\n"
"float64 des_q_w\n"
"\n"
"float64 des_thr\n"
"float64 thr2acc\n"
"float64 thr_scale_compensate\n"
"float64 voltage\n"
"\n"
"float64 err_axisang_x\n"
"float64 err_axisang_y\n"
"float64 err_axisang_z\n"
"float64 err_axisang_ang\n"
"\n"
"float64 fb_rate_x\n"
"float64 fb_rate_y\n"
"float64 fb_rate_z\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.des_p_x);
      stream.next(m.des_p_y);
      stream.next(m.des_p_z);
      stream.next(m.des_v_x);
      stream.next(m.des_v_y);
      stream.next(m.des_v_z);
      stream.next(m.fb_a_x);
      stream.next(m.fb_a_y);
      stream.next(m.fb_a_z);
      stream.next(m.des_a_x);
      stream.next(m.des_a_y);
      stream.next(m.des_a_z);
      stream.next(m.des_q_x);
      stream.next(m.des_q_y);
      stream.next(m.des_q_z);
      stream.next(m.des_q_w);
      stream.next(m.des_thr);
      stream.next(m.thr2acc);
      stream.next(m.thr_scale_compensate);
      stream.next(m.voltage);
      stream.next(m.err_axisang_x);
      stream.next(m.err_axisang_y);
      stream.next(m.err_axisang_z);
      stream.next(m.err_axisang_ang);
      stream.next(m.fb_rate_x);
      stream.next(m.fb_rate_y);
      stream.next(m.fb_rate_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Px4ctrlDebug_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::quadrotor_msgs::Px4ctrlDebug_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "des_p_x: ";
    Printer<double>::stream(s, indent + "  ", v.des_p_x);
    s << indent << "des_p_y: ";
    Printer<double>::stream(s, indent + "  ", v.des_p_y);
    s << indent << "des_p_z: ";
    Printer<double>::stream(s, indent + "  ", v.des_p_z);
    s << indent << "des_v_x: ";
    Printer<double>::stream(s, indent + "  ", v.des_v_x);
    s << indent << "des_v_y: ";
    Printer<double>::stream(s, indent + "  ", v.des_v_y);
    s << indent << "des_v_z: ";
    Printer<double>::stream(s, indent + "  ", v.des_v_z);
    s << indent << "fb_a_x: ";
    Printer<double>::stream(s, indent + "  ", v.fb_a_x);
    s << indent << "fb_a_y: ";
    Printer<double>::stream(s, indent + "  ", v.fb_a_y);
    s << indent << "fb_a_z: ";
    Printer<double>::stream(s, indent + "  ", v.fb_a_z);
    s << indent << "des_a_x: ";
    Printer<double>::stream(s, indent + "  ", v.des_a_x);
    s << indent << "des_a_y: ";
    Printer<double>::stream(s, indent + "  ", v.des_a_y);
    s << indent << "des_a_z: ";
    Printer<double>::stream(s, indent + "  ", v.des_a_z);
    s << indent << "des_q_x: ";
    Printer<double>::stream(s, indent + "  ", v.des_q_x);
    s << indent << "des_q_y: ";
    Printer<double>::stream(s, indent + "  ", v.des_q_y);
    s << indent << "des_q_z: ";
    Printer<double>::stream(s, indent + "  ", v.des_q_z);
    s << indent << "des_q_w: ";
    Printer<double>::stream(s, indent + "  ", v.des_q_w);
    s << indent << "des_thr: ";
    Printer<double>::stream(s, indent + "  ", v.des_thr);
    s << indent << "thr2acc: ";
    Printer<double>::stream(s, indent + "  ", v.thr2acc);
    s << indent << "thr_scale_compensate: ";
    Printer<double>::stream(s, indent + "  ", v.thr_scale_compensate);
    s << indent << "voltage: ";
    Printer<double>::stream(s, indent + "  ", v.voltage);
    s << indent << "err_axisang_x: ";
    Printer<double>::stream(s, indent + "  ", v.err_axisang_x);
    s << indent << "err_axisang_y: ";
    Printer<double>::stream(s, indent + "  ", v.err_axisang_y);
    s << indent << "err_axisang_z: ";
    Printer<double>::stream(s, indent + "  ", v.err_axisang_z);
    s << indent << "err_axisang_ang: ";
    Printer<double>::stream(s, indent + "  ", v.err_axisang_ang);
    s << indent << "fb_rate_x: ";
    Printer<double>::stream(s, indent + "  ", v.fb_rate_x);
    s << indent << "fb_rate_y: ";
    Printer<double>::stream(s, indent + "  ", v.fb_rate_y);
    s << indent << "fb_rate_z: ";
    Printer<double>::stream(s, indent + "  ", v.fb_rate_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_PX4CTRLDEBUG_H