// Generated by gencpp from file transformtrack/TransformBatchResponse.msg
// DO NOT EDIT!


#ifndef TRANSFORMTRACK_MESSAGE_TRANSFORMBATCHRESPONSE_H
#define TRANSFORMTRACK_MESSAGE_TRANSFORMBATCHRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Float64MultiArray.h>

namespace transformtrack
{
template <class ContainerAllocator>
struct TransformBatchResponse_
{
  typedef TransformBatchResponse_<ContainerAllocator> Type;

  TransformBatchResponse_()
    : transforms()
    , distances()  {
    }
  TransformBatchResponse_(const ContainerAllocator& _alloc)
    : transforms(_alloc)
    , distances(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Float64MultiArray_<ContainerAllocator>  _transforms_type;
  _transforms_type transforms;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _distances_type;
  _distances_type distances;





  typedef boost::shared_ptr< ::transformtrack::TransformBatchResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::transformtrack::TransformBatchResponse_<ContainerAllocator> const> ConstPtr;

}; // struct TransformBatchResponse_

typedef ::transformtrack::TransformBatchResponse_<std::allocator<void> > TransformBatchResponse;

typedef boost::shared_ptr< ::transformtrack::TransformBatchResponse > TransformBatchResponsePtr;
typedef boost::shared_ptr< ::transformtrack::TransformBatchResponse const> TransformBatchResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::transformtrack::TransformBatchResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::transformtrack::TransformBatchResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::transformtrack::TransformBatchResponse_<ContainerAllocator1> & lhs, const ::transformtrack::TransformBatchResponse_<ContainerAllocator2> & rhs)
{
  return lhs.transforms == rhs.transforms &&
    lhs.distances == rhs.distances;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::transformtrack::TransformBatchResponse_<ContainerAllocator1> & lhs, const ::transformtrack::TransformBatchResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace transformtrack

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::transformtrack::TransformBatchResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::transformtrack::TransformBatchResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::transformtrack::TransformBatchResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::transformtrack::TransformBatchResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::transformtrack::TransformBatchResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::transformtrack::TransformBatchResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::transformtrack::TransformBatchResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ca8be60547a73646ae9c3cf9156c7ab7";
  }

  static const char* value(const ::transformtrack::TransformBatchResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xca8be60547a73646ULL;
  static const uint64_t static_value2 = 0xae9c3cf9156c7ab7ULL;
};

template<class ContainerAllocator>
struct DataType< ::transformtrack::TransformBatchResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "transformtrack/TransformBatchResponse";
  }

  static const char* value(const ::transformtrack::TransformBatchResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::transformtrack::TransformBatchResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Float64MultiArray transforms\n"
"float64[] distances\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Float64MultiArray\n"
"# Please look at the MultiArrayLayout message definition for\n"
"# documentation on all multiarrays.\n"
"\n"
"MultiArrayLayout  layout        # specification of data layout\n"
"float64[]         data          # array of data\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/MultiArrayLayout\n"
"# The multiarray declares a generic multi-dimensional array of a\n"
"# particular data type.  Dimensions are ordered from outer most\n"
"# to inner most.\n"
"\n"
"MultiArrayDimension[] dim # Array of dimension properties\n"
"uint32 data_offset        # padding elements at front of data\n"
"\n"
"# Accessors should ALWAYS be written in terms of dimension stride\n"
"# and specified outer-most dimension first.\n"
"# \n"
"# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n"
"#\n"
"# A standard, 3-channel 640x480 image with interleaved color channels\n"
"# would be specified as:\n"
"#\n"
"# dim[0].label  = \"height\"\n"
"# dim[0].size   = 480\n"
"# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n"
"# dim[1].label  = \"width\"\n"
"# dim[1].size   = 640\n"
"# dim[1].stride = 3*640 = 1920\n"
"# dim[2].label  = \"channel\"\n"
"# dim[2].size   = 3\n"
"# dim[2].stride = 3\n"
"#\n"
"# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/MultiArrayDimension\n"
"string label   # label of given dimension\n"
"uint32 size    # size of given dimension (in type units)\n"
"uint32 stride  # stride of given dimension\n"
;
  }

  static const char* value(const ::transformtrack::TransformBatchResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::transformtrack::TransformBatchResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.transforms);
      stream.next(m.distances);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TransformBatchResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::transformtrack::TransformBatchResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::transformtrack::TransformBatchResponse_<ContainerAllocator>& v)
  {
    s << indent << "transforms: ";
    s << std::endl;
    Printer< ::std_msgs::Float64MultiArray_<ContainerAllocator> >::stream(s, indent + "  ", v.transforms);
    s << indent << "distances[]" << std::endl;
    for (size_t i = 0; i < v.distances.size(); ++i)
    {
      s << indent << "  distances[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.distances[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRANSFORMTRACK_MESSAGE_TRANSFORMBATCHRESPONSE_H
