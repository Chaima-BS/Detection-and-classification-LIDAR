// Generated from file detection_classification/trackbox.msg



#ifndef MESSAGE_TRACKBOX_H
#define MESSAGE_TRACKBOX_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace detection_classification
{
template <class ContainerAllocator>
struct trackbox_
{
  typedef trackbox_<ContainerAllocator> Type;

  trackbox_()
    : header()
    , box_num(0)
    , x1()
    , x2()
    , x3()
    , x4()
    , y1()
    , y2()
    , y3()
    , y4()  {
    }
  trackbox_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , box_num(0)
    , x1(_alloc)
    , x2(_alloc)
    , x3(_alloc)
    , x4(_alloc)
    , y1(_alloc)
    , y2(_alloc)
    , y3(_alloc)
    , y4(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _box_num_type;
  _box_num_type box_num;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _x1_type;
  _x1_type x1;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _x2_type;
  _x2_type x2;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _x3_type;
  _x3_type x3;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _x4_type;
  _x4_type x4;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _y1_type;
  _y1_type y1;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _y2_type;
  _y2_type y2;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _y3_type;
  _y3_type y3;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _y4_type;
  _y4_type y4;





  typedef boost::shared_ptr< ::detection_classification::trackbox_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::detection_classification::trackbox_<ContainerAllocator> const> ConstPtr;

}; // struct trackbox_

typedef ::detection_classification::trackbox_<std::allocator<void> > trackbox;

typedef boost::shared_ptr< ::detection_classification::trackbox > trackboxPtr;
typedef boost::shared_ptr< ::detection_classification::trackbox const> trackboxConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::detection_classification::trackbox_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::detection_classification::trackbox_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::detection_classification::trackbox_<ContainerAllocator1> & lhs, const ::detection_classification::trackbox_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.box_num == rhs.box_num &&
    lhs.x1 == rhs.x1 &&
    lhs.x2 == rhs.x2 &&
    lhs.x3 == rhs.x3 &&
    lhs.x4 == rhs.x4 &&
    lhs.y1 == rhs.y1 &&
    lhs.y2 == rhs.y2 &&
    lhs.y3 == rhs.y3 &&
    lhs.y4 == rhs.y4;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::detection_classification::trackbox_<ContainerAllocator1> & lhs, const ::detection_classification::trackbox_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace detection_classification

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::detection_classification::trackbox_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::detection_classification::trackbox_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::detection_classification::trackbox_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::detection_classification::trackbox_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::detection_classification::trackbox_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::detection_classification::trackbox_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::detection_classification::trackbox_<ContainerAllocator> >
{
  static const char* value()
  {
    return "abfa3e829554679ea1d12754b272bb49";
  }

  static const char* value(const ::detection_classification::trackbox_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xabfa3e829554679eULL;
  static const uint64_t static_value2 = 0xa1d12754b272bb49ULL;
};

template<class ContainerAllocator>
struct DataType< ::detection_classification::trackbox_<ContainerAllocator> >
{
  static const char* value()
  {
    return "detection_classification/trackbox";
  }

  static const char* value(const ::detection_classification::trackbox_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::detection_classification::trackbox_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"uint8  box_num\n"
"float32[] x1\n"
"float32[] x2\n"
"float32[] x3\n"
"float32[] x4\n"
"float32[] y1\n"
"float32[] y2\n"
"float32[] y3\n"
"float32[] y4\n"
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

  static const char* value(const ::detection_classification::trackbox_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::detection_classification::trackbox_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.box_num);
      stream.next(m.x1);
      stream.next(m.x2);
      stream.next(m.x3);
      stream.next(m.x4);
      stream.next(m.y1);
      stream.next(m.y2);
      stream.next(m.y3);
      stream.next(m.y4);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct trackbox_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::detection_classification::trackbox_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::detection_classification::trackbox_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "box_num: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.box_num);
    s << indent << "x1[]" << std::endl;
    for (size_t i = 0; i < v.x1.size(); ++i)
    {
      s << indent << "  x1[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.x1[i]);
    }
    s << indent << "x2[]" << std::endl;
    for (size_t i = 0; i < v.x2.size(); ++i)
    {
      s << indent << "  x2[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.x2[i]);
    }
    s << indent << "x3[]" << std::endl;
    for (size_t i = 0; i < v.x3.size(); ++i)
    {
      s << indent << "  x3[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.x3[i]);
    }
    s << indent << "x4[]" << std::endl;
    for (size_t i = 0; i < v.x4.size(); ++i)
    {
      s << indent << "  x4[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.x4[i]);
    }
    s << indent << "y1[]" << std::endl;
    for (size_t i = 0; i < v.y1.size(); ++i)
    {
      s << indent << "  y1[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.y1[i]);
    }
    s << indent << "y2[]" << std::endl;
    for (size_t i = 0; i < v.y2.size(); ++i)
    {
      s << indent << "  y2[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.y2[i]);
    }
    s << indent << "y3[]" << std::endl;
    for (size_t i = 0; i < v.y3.size(); ++i)
    {
      s << indent << "  y3[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.y3[i]);
    }
    s << indent << "y4[]" << std::endl;
    for (size_t i = 0; i < v.y4.size(); ++i)
    {
      s << indent << "  y4[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.y4[i]);
    }
  }
};

} 
} 

#endif // MESSAGE_TRACKBOX_H