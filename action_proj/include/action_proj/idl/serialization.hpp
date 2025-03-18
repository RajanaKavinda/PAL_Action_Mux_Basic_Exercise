#ifndef ROS_BABEL_FISH_SERIALIZATION_HPP
#define ROS_BABEL_FISH_SERIALIZATION_HPP

#include "action_proj/idl/type_support.hpp"
#include "action_proj/messages/message.hpp"

#include <rclcpp/serialized_message.hpp>

namespace ros_babel_fish
{
std::shared_ptr<void>
createContainer( const rosidl_typesupport_introspection_cpp::MessageMembers &members,
                 rosidl_runtime_cpp::MessageInitialization initialization =
                     rosidl_runtime_cpp::MessageInitialization::ALL );

std::shared_ptr<void> createContainer( const MessageMembersIntrospection &members,
                                       rosidl_runtime_cpp::MessageInitialization initialization =
                                           rosidl_runtime_cpp::MessageInitialization::ALL );
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_SERIALIZATION_HPP
