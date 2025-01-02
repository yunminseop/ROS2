// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from control_msgs:srv/MultiSpawn.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__SRV__DETAIL__MULTI_SPAWN__TRAITS_HPP_
#define CONTROL_MSGS__SRV__DETAIL__MULTI_SPAWN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "control_msgs/srv/detail/multi_spawn__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace control_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MultiSpawn_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: num
  {
    out << "num: ";
    rosidl_generator_traits::value_to_yaml(msg.num, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MultiSpawn_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num: ";
    rosidl_generator_traits::value_to_yaml(msg.num, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MultiSpawn_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace control_msgs

namespace rosidl_generator_traits
{

[[deprecated("use control_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control_msgs::srv::MultiSpawn_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const control_msgs::srv::MultiSpawn_Request & msg)
{
  return control_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<control_msgs::srv::MultiSpawn_Request>()
{
  return "control_msgs::srv::MultiSpawn_Request";
}

template<>
inline const char * name<control_msgs::srv::MultiSpawn_Request>()
{
  return "control_msgs/srv/MultiSpawn_Request";
}

template<>
struct has_fixed_size<control_msgs::srv::MultiSpawn_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<control_msgs::srv::MultiSpawn_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<control_msgs::srv::MultiSpawn_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace control_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const MultiSpawn_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    if (msg.x.size() == 0) {
      out << "x: []";
    } else {
      out << "x: [";
      size_t pending_items = msg.x.size();
      for (auto item : msg.x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: y
  {
    if (msg.y.size() == 0) {
      out << "y: []";
    } else {
      out << "y: [";
      size_t pending_items = msg.y.size();
      for (auto item : msg.y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: theta
  {
    if (msg.theta.size() == 0) {
      out << "theta: []";
    } else {
      out << "theta: [";
      size_t pending_items = msg.theta.size();
      for (auto item : msg.theta) {
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
  const MultiSpawn_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.x.size() == 0) {
      out << "x: []\n";
    } else {
      out << "x:\n";
      for (auto item : msg.x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.y.size() == 0) {
      out << "y: []\n";
    } else {
      out << "y:\n";
      for (auto item : msg.y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.theta.size() == 0) {
      out << "theta: []\n";
    } else {
      out << "theta:\n";
      for (auto item : msg.theta) {
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

inline std::string to_yaml(const MultiSpawn_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace control_msgs

namespace rosidl_generator_traits
{

[[deprecated("use control_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const control_msgs::srv::MultiSpawn_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  control_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use control_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const control_msgs::srv::MultiSpawn_Response & msg)
{
  return control_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<control_msgs::srv::MultiSpawn_Response>()
{
  return "control_msgs::srv::MultiSpawn_Response";
}

template<>
inline const char * name<control_msgs::srv::MultiSpawn_Response>()
{
  return "control_msgs/srv/MultiSpawn_Response";
}

template<>
struct has_fixed_size<control_msgs::srv::MultiSpawn_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<control_msgs::srv::MultiSpawn_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<control_msgs::srv::MultiSpawn_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<control_msgs::srv::MultiSpawn>()
{
  return "control_msgs::srv::MultiSpawn";
}

template<>
inline const char * name<control_msgs::srv::MultiSpawn>()
{
  return "control_msgs/srv/MultiSpawn";
}

template<>
struct has_fixed_size<control_msgs::srv::MultiSpawn>
  : std::integral_constant<
    bool,
    has_fixed_size<control_msgs::srv::MultiSpawn_Request>::value &&
    has_fixed_size<control_msgs::srv::MultiSpawn_Response>::value
  >
{
};

template<>
struct has_bounded_size<control_msgs::srv::MultiSpawn>
  : std::integral_constant<
    bool,
    has_bounded_size<control_msgs::srv::MultiSpawn_Request>::value &&
    has_bounded_size<control_msgs::srv::MultiSpawn_Response>::value
  >
{
};

template<>
struct is_service<control_msgs::srv::MultiSpawn>
  : std::true_type
{
};

template<>
struct is_service_request<control_msgs::srv::MultiSpawn_Request>
  : std::true_type
{
};

template<>
struct is_service_response<control_msgs::srv::MultiSpawn_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROL_MSGS__SRV__DETAIL__MULTI_SPAWN__TRAITS_HPP_
