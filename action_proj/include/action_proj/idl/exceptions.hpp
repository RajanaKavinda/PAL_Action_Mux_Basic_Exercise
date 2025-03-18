#ifndef ROS_BABEL_FISH_IDL_EXCEPTIONS_HPP
#define ROS_BABEL_FISH_IDL_EXCEPTIONS_HPP

#include "action_proj/exceptions/babel_fish_exception.hpp"

namespace ros_babel_fish
{

class TypeSupportException : public BabelFishException
{
public:
  using BabelFishException::BabelFishException;
};
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_IDL_EXCEPTIONS_HPP
