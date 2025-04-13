// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from msg_interfaces:msg/WristJointVelocity.idl
// generated code does not contain a copyright notice

#ifndef MSG_INTERFACES__MSG__DETAIL__WRIST_JOINT_VELOCITY__TRAITS_HPP_
#define MSG_INTERFACES__MSG__DETAIL__WRIST_JOINT_VELOCITY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "msg_interfaces/msg/detail/wrist_joint_velocity__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace msg_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const WristJointVelocity & msg,
  std::ostream & out)
{
  out << "{";
  // member: angular_speed
  {
    if (msg.angular_speed.size() == 0) {
      out << "angular_speed: []";
    } else {
      out << "angular_speed: [";
      size_t pending_items = msg.angular_speed.size();
      for (auto item : msg.angular_speed) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WristJointVelocity & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angular_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.angular_speed.size() == 0) {
      out << "angular_speed: []\n";
    } else {
      out << "angular_speed:\n";
      for (auto item : msg.angular_speed) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WristJointVelocity & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace msg_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use msg_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const msg_interfaces::msg::WristJointVelocity & msg,
  std::ostream & out, size_t indentation = 0)
{
  msg_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use msg_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const msg_interfaces::msg::WristJointVelocity & msg)
{
  return msg_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<msg_interfaces::msg::WristJointVelocity>()
{
  return "msg_interfaces::msg::WristJointVelocity";
}

template<>
inline const char * name<msg_interfaces::msg::WristJointVelocity>()
{
  return "msg_interfaces/msg/WristJointVelocity";
}

template<>
struct has_fixed_size<msg_interfaces::msg::WristJointVelocity>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<msg_interfaces::msg::WristJointVelocity>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<msg_interfaces::msg::WristJointVelocity>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MSG_INTERFACES__MSG__DETAIL__WRIST_JOINT_VELOCITY__TRAITS_HPP_
