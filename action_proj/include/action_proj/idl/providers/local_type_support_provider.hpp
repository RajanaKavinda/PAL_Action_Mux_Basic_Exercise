#ifndef ROS_BABEL_FISH_INTEGRATED_DESCRIPTION_PROVIDER_H
#define ROS_BABEL_FISH_INTEGRATED_DESCRIPTION_PROVIDER_H

#include "action_proj/idl/type_support_provider.hpp"

namespace ros_babel_fish
{

/**
 * @brief Looks up message libraries that are available locally on the machine.
 */
class LocalTypeSupportProvider : public TypeSupportProvider
{
public:
  LocalTypeSupportProvider();

protected:
  MessageTypeSupport::ConstSharedPtr getMessageTypeSupportImpl( const std::string &type ) const override;

  ServiceTypeSupport::ConstSharedPtr getServiceTypeSupportImpl( const std::string &type ) const override;

  ActionTypeSupport::ConstSharedPtr getActionTypeSupportImpl( const std::string &type ) const override;
};
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_INTEGRATED_DESCRIPTION_PROVIDER_H
