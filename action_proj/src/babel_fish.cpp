#include "action_proj/babel_fish.hpp"

#include "logging.hpp"
#include "action_proj/detail/babel_fish_service_client.hpp"
#include "action_proj/detail/topic.hpp"
#include "action_proj/exceptions/babel_fish_exception.hpp"
#include "action_proj/idl/providers/local_type_support_provider.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/wait_set.hpp>

namespace ros_babel_fish
{

BabelFish::BabelFish()
{
  type_support_providers_.push_back( std::make_shared<LocalTypeSupportProvider>() );
}

BabelFish::BabelFish( std::vector<TypeSupportProvider::SharedPtr> type_support_providers )
    : type_support_providers_( std::move( type_support_providers ) )
{
}

BabelFish::~BabelFish() = default;

BabelFishSubscription::SharedPtr BabelFish::create_subscription(
    rclcpp::Node &node, const std::string &topic, const rclcpp::QoS &qos,
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
    rclcpp::CallbackGroup::SharedPtr group, rclcpp::SubscriptionOptions options,
    std::chrono::nanoseconds timeout )
{
  const std::string &resolved_topic = resolve_topic( node, topic );
  std::vector<std::string> types;
  if ( !wait_for_topic_and_type( node, resolved_topic, types, timeout ) )
    return nullptr;
  if ( types.empty() ) {
    RBF2_ERROR( "Could not subscribe to '%s'.Topic is available but has no type!",
                resolved_topic.c_str() );
    return nullptr;
  }

  if ( types.size() > 1 ) {
    RBF2_INFO( "Topic '%s' has more than one type. Selecting the first arbitrarily: '%s'.",
               resolved_topic.c_str(), types[0].c_str() );
  }

  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support( types[0] );
  if ( type_support == nullptr ) {
    throw BabelFishException( "Failed to create a subscriber for type: " + types[0] +
                              ". Type not found!" );
  }
  try {
    // TODO Add support for topic statistics?
    auto subscription = std::make_shared<BabelFishSubscription>(
        node.get_node_base_interface().get(), type_support, topic, qos, std::move( callback ),
        std::move( options ) );
    node.get_node_topics_interface()->add_subscription( subscription, std::move( group ) );
    return subscription;
  } catch ( const std::runtime_error &ex ) {
    throw BabelFishException( "Failed to create Subscription: " + std::string( ex.what() ) );
  }
}

BabelFishSubscription::SharedPtr BabelFish::create_subscription(
    rclcpp::Node &node, const std::string &topic, const std::string &type, const rclcpp::QoS &qos,
    rclcpp::AnySubscriptionCallback<CompoundMessage, std::allocator<void>> callback,
    rclcpp::CallbackGroup::SharedPtr group, rclcpp::SubscriptionOptions options )
{
  const std::string &resolved_topic = resolve_topic( node, topic );

  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "Failed to create a subscriber for type: " + type +
                              ". Type not found!" );
  }
  try {
    auto subscription = std::make_shared<BabelFishSubscription>(
        node.get_node_base_interface().get(), type_support, topic, qos, std::move( callback ),
        std::move( options ) );

    node.get_node_topics_interface()->add_subscription( subscription, std::move( group ) );
    return subscription;
  } catch ( const std::runtime_error &ex ) {
    throw BabelFishException( "Failed to create Subscription: " + std::string( ex.what() ) );
  }
}

CompoundMessage BabelFish::create_message( const std::string &type ) const
{
  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "BabelFish doesn't know a message of type: " + type );
  }
  return CompoundMessage( *type_support );
}

CompoundMessage::SharedPtr BabelFish::create_message_shared( const std::string &type ) const
{
  MessageTypeSupport::ConstSharedPtr type_support = get_message_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "BabelFish doesn't know a message of type: " + type );
  }
  return CompoundMessage::make_shared( *type_support );
}

CompoundMessage BabelFish::create_service_request( const std::string &type ) const
{
  const ServiceTypeSupport::ConstSharedPtr &type_support = get_service_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "BabelFish doesn't know a service of type: " + type );
  }
  return CompoundMessage( type_support->request() );
}

CompoundMessage::SharedPtr BabelFish::create_service_request_shared( const std::string &type ) const
{
  const ServiceTypeSupport::ConstSharedPtr &type_support = get_service_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "BabelFish doesn't know a service of type: " + type );
  }
  return CompoundMessage::make_shared( type_support->request() );
}

CompoundMessage BabelFish::create_action_goal( const std::string &type ) const
{
  const ActionTypeSupport::ConstSharedPtr &type_support = get_action_type_support( type );
  if ( type_support == nullptr ) {
    throw BabelFishException( "BabelFish doesn't know an action of type: " + type );
  }
  MessageMembersIntrospection introspection = type_support->goal_service_type_support->request();
  size_t index =
      std::find_if( introspection->members_, introspection->members_ + introspection->member_count_,
                    []( const auto &a ) { return std::strcmp( a.name_, "goal" ) == 0; } ) -
      introspection->members_;
  return CompoundMessage( type_support->goal_service_type_support->request().getMember( index ) );
}

CompoundMessage::SharedPtr BabelFish::create_action_goal_shared( const std::string &type ) const
{
  return CompoundMessage::make_shared( create_action_goal( type ) );
}

MessageTypeSupport::ConstSharedPtr BabelFish::get_message_type_support( const std::string &type ) const
{
  for ( const auto &provider : type_support_providers_ ) {
    MessageTypeSupport::ConstSharedPtr result = provider->getMessageTypeSupport( type );
    if ( result == nullptr )
      continue;
    return result;
  }
  return nullptr;
}

ServiceTypeSupport::ConstSharedPtr BabelFish::get_service_type_support( const std::string &type ) const
{
  for ( const auto &provider : type_support_providers_ ) {
    ServiceTypeSupport::ConstSharedPtr result = provider->getServiceTypeSupport( type );
    if ( result == nullptr )
      continue;
    return result;
  }
  return nullptr;
}

ActionTypeSupport::ConstSharedPtr BabelFish::get_action_type_support( const std::string &type ) const
{
  for ( const auto &provider : type_support_providers_ ) {
    ActionTypeSupport::ConstSharedPtr result = provider->getActionTypeSupport( type );
    if ( result == nullptr )
      continue;
    return result;
  }
  return nullptr;
}

std::vector<TypeSupportProvider::SharedPtr> BabelFish::type_support_providers()
{
  return type_support_providers_;
}
} // namespace ros_babel_fish
