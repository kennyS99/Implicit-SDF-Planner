// Generated by gencpp from file traj_utils/PolyTraj.msg
// DO NOT EDIT!


#ifndef TRAJ_UTILS_MESSAGE_POLYTRAJ_H
#define TRAJ_UTILS_MESSAGE_POLYTRAJ_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace traj_utils
{
template <class ContainerAllocator>
struct PolyTraj_
{
  typedef PolyTraj_<ContainerAllocator> Type;

  PolyTraj_()
    : drone_id(0)
    , traj_id(0)
    , start_time()
    , order(0)
    , coef_x()
    , coef_y()
    , coef_z()
    , duration()  {
    }
  PolyTraj_(const ContainerAllocator& _alloc)
    : drone_id(0)
    , traj_id(0)
    , start_time()
    , order(0)
    , coef_x(_alloc)
    , coef_y(_alloc)
    , coef_z(_alloc)
    , duration(_alloc)  {
  (void)_alloc;
    }



   typedef int16_t _drone_id_type;
  _drone_id_type drone_id;

   typedef int32_t _traj_id_type;
  _traj_id_type traj_id;

   typedef ros::Time _start_time_type;
  _start_time_type start_time;

   typedef uint8_t _order_type;
  _order_type order;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _coef_x_type;
  _coef_x_type coef_x;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _coef_y_type;
  _coef_y_type coef_y;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _coef_z_type;
  _coef_z_type coef_z;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _duration_type;
  _duration_type duration;





  typedef boost::shared_ptr< ::traj_utils::PolyTraj_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::traj_utils::PolyTraj_<ContainerAllocator> const> ConstPtr;

}; // struct PolyTraj_

typedef ::traj_utils::PolyTraj_<std::allocator<void> > PolyTraj;

typedef boost::shared_ptr< ::traj_utils::PolyTraj > PolyTrajPtr;
typedef boost::shared_ptr< ::traj_utils::PolyTraj const> PolyTrajConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::traj_utils::PolyTraj_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::traj_utils::PolyTraj_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::traj_utils::PolyTraj_<ContainerAllocator1> & lhs, const ::traj_utils::PolyTraj_<ContainerAllocator2> & rhs)
{
  return lhs.drone_id == rhs.drone_id &&
    lhs.traj_id == rhs.traj_id &&
    lhs.start_time == rhs.start_time &&
    lhs.order == rhs.order &&
    lhs.coef_x == rhs.coef_x &&
    lhs.coef_y == rhs.coef_y &&
    lhs.coef_z == rhs.coef_z &&
    lhs.duration == rhs.duration;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::traj_utils::PolyTraj_<ContainerAllocator1> & lhs, const ::traj_utils::PolyTraj_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace traj_utils

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::traj_utils::PolyTraj_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::traj_utils::PolyTraj_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traj_utils::PolyTraj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traj_utils::PolyTraj_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traj_utils::PolyTraj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traj_utils::PolyTraj_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::traj_utils::PolyTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7d47340fc475c0357eff322fb6ab494d";
  }

  static const char* value(const ::traj_utils::PolyTraj_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7d47340fc475c035ULL;
  static const uint64_t static_value2 = 0x7eff322fb6ab494dULL;
};

template<class ContainerAllocator>
struct DataType< ::traj_utils::PolyTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "traj_utils/PolyTraj";
  }

  static const char* value(const ::traj_utils::PolyTraj_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::traj_utils::PolyTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 drone_id\n"
"int32 traj_id\n"
"time start_time\n"
"\n"
"uint8 order\n"
"float32[] coef_x\n"
"float32[] coef_y\n"
"float32[] coef_z\n"
"float32[] duration\n"
;
  }

  static const char* value(const ::traj_utils::PolyTraj_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::traj_utils::PolyTraj_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.drone_id);
      stream.next(m.traj_id);
      stream.next(m.start_time);
      stream.next(m.order);
      stream.next(m.coef_x);
      stream.next(m.coef_y);
      stream.next(m.coef_z);
      stream.next(m.duration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PolyTraj_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::traj_utils::PolyTraj_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::traj_utils::PolyTraj_<ContainerAllocator>& v)
  {
    s << indent << "drone_id: ";
    Printer<int16_t>::stream(s, indent + "  ", v.drone_id);
    s << indent << "traj_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.traj_id);
    s << indent << "start_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.start_time);
    s << indent << "order: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.order);
    s << indent << "coef_x[]" << std::endl;
    for (size_t i = 0; i < v.coef_x.size(); ++i)
    {
      s << indent << "  coef_x[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.coef_x[i]);
    }
    s << indent << "coef_y[]" << std::endl;
    for (size_t i = 0; i < v.coef_y.size(); ++i)
    {
      s << indent << "  coef_y[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.coef_y[i]);
    }
    s << indent << "coef_z[]" << std::endl;
    for (size_t i = 0; i < v.coef_z.size(); ++i)
    {
      s << indent << "  coef_z[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.coef_z[i]);
    }
    s << indent << "duration[]" << std::endl;
    for (size_t i = 0; i < v.duration.size(); ++i)
    {
      s << indent << "  duration[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.duration[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRAJ_UTILS_MESSAGE_POLYTRAJ_H
