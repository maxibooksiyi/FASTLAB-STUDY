// Generated by gencpp from file vicon_bridge/viconCalibrateSegment.msg
// DO NOT EDIT!


#ifndef VICON_BRIDGE_MESSAGE_VICONCALIBRATESEGMENT_H
#define VICON_BRIDGE_MESSAGE_VICONCALIBRATESEGMENT_H

#include <ros/service_traits.h>


#include <vicon_bridge/viconCalibrateSegmentRequest.h>
#include <vicon_bridge/viconCalibrateSegmentResponse.h>


namespace vicon_bridge
{

struct viconCalibrateSegment
{

typedef viconCalibrateSegmentRequest Request;
typedef viconCalibrateSegmentResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct viconCalibrateSegment
} // namespace vicon_bridge


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::vicon_bridge::viconCalibrateSegment > {
  static const char* value()
  {
    return "ca1b34be858a36828126364b1a577794";
  }

  static const char* value(const ::vicon_bridge::viconCalibrateSegment&) { return value(); }
};

template<>
struct DataType< ::vicon_bridge::viconCalibrateSegment > {
  static const char* value()
  {
    return "vicon_bridge/viconCalibrateSegment";
  }

  static const char* value(const ::vicon_bridge::viconCalibrateSegment&) { return value(); }
};


// service_traits::MD5Sum< ::vicon_bridge::viconCalibrateSegmentRequest> should match
// service_traits::MD5Sum< ::vicon_bridge::viconCalibrateSegment >
template<>
struct MD5Sum< ::vicon_bridge::viconCalibrateSegmentRequest>
{
  static const char* value()
  {
    return MD5Sum< ::vicon_bridge::viconCalibrateSegment >::value();
  }
  static const char* value(const ::vicon_bridge::viconCalibrateSegmentRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::vicon_bridge::viconCalibrateSegmentRequest> should match
// service_traits::DataType< ::vicon_bridge::viconCalibrateSegment >
template<>
struct DataType< ::vicon_bridge::viconCalibrateSegmentRequest>
{
  static const char* value()
  {
    return DataType< ::vicon_bridge::viconCalibrateSegment >::value();
  }
  static const char* value(const ::vicon_bridge::viconCalibrateSegmentRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::vicon_bridge::viconCalibrateSegmentResponse> should match
// service_traits::MD5Sum< ::vicon_bridge::viconCalibrateSegment >
template<>
struct MD5Sum< ::vicon_bridge::viconCalibrateSegmentResponse>
{
  static const char* value()
  {
    return MD5Sum< ::vicon_bridge::viconCalibrateSegment >::value();
  }
  static const char* value(const ::vicon_bridge::viconCalibrateSegmentResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::vicon_bridge::viconCalibrateSegmentResponse> should match
// service_traits::DataType< ::vicon_bridge::viconCalibrateSegment >
template<>
struct DataType< ::vicon_bridge::viconCalibrateSegmentResponse>
{
  static const char* value()
  {
    return DataType< ::vicon_bridge::viconCalibrateSegment >::value();
  }
  static const char* value(const ::vicon_bridge::viconCalibrateSegmentResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // VICON_BRIDGE_MESSAGE_VICONCALIBRATESEGMENT_H
