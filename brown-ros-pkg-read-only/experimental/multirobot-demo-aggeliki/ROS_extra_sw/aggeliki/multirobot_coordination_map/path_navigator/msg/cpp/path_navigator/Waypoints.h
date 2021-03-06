/* auto-generated by genmsg_cpp from /opt/ros/boxturtle/ros/aggeliki/path_navigator/msg/Waypoints.msg.  Do not edit! */
#ifndef PATH_NAVIGATOR_WAYPOINTS_H
#define PATH_NAVIGATOR_WAYPOINTS_H

#include <string>
#include <vector>
#include "ros/message.h"
#include "ros/debug.h"
#include "ros/assert.h"
#include "ros/time.h"

#include "roslib/Header.h"
#include "position_tracker/Position.h"
#include "map_loader/Node.h"

namespace path_navigator
{

//! \htmlinclude Waypoints.msg.html

class Waypoints : public ros::Message
{
public:
  typedef boost::shared_ptr<Waypoints> Ptr;
  typedef boost::shared_ptr<Waypoints const> ConstPtr;

  typedef std::vector<map_loader::Node> _waypoints_type;

  std::vector<map_loader::Node> waypoints;

  Waypoints() : ros::Message()
  {
  }
  Waypoints(const Waypoints &copy) : ros::Message()
  {
    (void)copy;
    waypoints = copy.waypoints;
  }
  Waypoints &operator =(const Waypoints &copy)
  {
    if (this == &copy)
      return *this;
    waypoints.clear();
    waypoints = copy.waypoints;
    return *this;
  }
  virtual ~Waypoints() 
  {
    waypoints.clear();
  }
  inline static std::string __s_getDataType() { return std::string("path_navigator/Waypoints"); }
  inline static std::string __s_getMD5Sum() { return std::string("8bc19531595b774aa17fbc5b24e4bd79"); }
  inline static std::string __s_getMessageDefinition()
  {
    return std::string(
    "map_loader/Node[] waypoints\n"
    "\n"
    "================================================================================\n"
    "MSG: map_loader/Node\n"
    "int32 id\n"
    "#Node previous\n"
    "int32 distanceFromStart\n"
    "position_tracker/Position p\n"
    "\n"
    "\n"
    "================================================================================\n"
    "MSG: position_tracker/Position\n"
    "Header header\n"
    "float64 x\n"
    "float64 y\n"
    "float64 theta\n"
    "\n"
    "================================================================================\n"
    "MSG: roslib/Header\n"
    "# Standard metadata for higher-level stamped data types.\n"
    "# This is generally used to communicate timestamped data \n"
    "# in a particular coordinate frame.\n"
    "# \n"
    "# sequence ID: consecutively increasing ID \n"
    "uint32 seq\n"
    "#Two-integer timestamp that is expressed as:\n"
    "# * stamp.secs: seconds (stamp_secs) since epoch\n"
    "# * stamp.nsecs: nanoseconds since stamp_secs\n"
    "# time-handling sugar is provided by the client library\n"
    "time stamp\n"
    "#Frame this data is associated with\n"
    "# 0: no frame\n"
    "# 1: global frame\n"
    "string frame_id\n"
    "\n"
    "\n"
    );
  }
  inline virtual const std::string __getDataType() const { return __s_getDataType(); }
  inline virtual const std::string __getMD5Sum() const { return __s_getMD5Sum(); }
  inline virtual const std::string __getMessageDefinition() const { return __s_getMessageDefinition(); }
  void set_waypoints_size(uint32_t __ros_new_size)
  {
    this->waypoints.resize(__ros_new_size);
  }
  inline uint32_t get_waypoints_size() const { return waypoints.size(); }
  uint32_t calc_waypoints_array_serialization_len() const
  {
    uint32_t l = 0;
    uint32_t waypoints_size = waypoints.size();
    for (size_t i = 0; i < waypoints_size; i++)
      l += waypoints[i].serializationLength();
    return l;
  }
  inline void get_waypoints_vec (std::vector<map_loader::Node> &__ros_vec) const
  {
    __ros_vec = this->waypoints;
  }
  inline void set_waypoints_vec(const std::vector<map_loader::Node> &__ros_vec)
  {
    this->waypoints = __ros_vec;
  }
  inline uint32_t serializationLength() const
  {
    unsigned __l = 0;
    __l += 4 + calc_waypoints_array_serialization_len(); // waypoints
    return __l;
  }
  virtual uint8_t *serialize(uint8_t *write_ptr,
#if defined(__GNUC__)
                             __attribute__((unused)) uint32_t seq) const
#else
                             uint32_t seq) const
#endif
  {
    uint32_t __waypoints_size = waypoints.size();
    SROS_SERIALIZE_PRIMITIVE(write_ptr, __waypoints_size);
    for (size_t i = 0; i < __waypoints_size; i++)
      write_ptr = waypoints[i].serialize(write_ptr, seq);
    return write_ptr;
  }
  virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    uint32_t __waypoints_size;
    SROS_DESERIALIZE_PRIMITIVE(read_ptr, __waypoints_size);
    set_waypoints_size(__waypoints_size);
    for (size_t i = 0; i < __waypoints_size; i++)
      read_ptr = waypoints[i].deserialize(read_ptr);
    return read_ptr;
  }
};

typedef boost::shared_ptr<Waypoints> WaypointsPtr;
typedef boost::shared_ptr<Waypoints const> WaypointsConstPtr;


}

#endif
