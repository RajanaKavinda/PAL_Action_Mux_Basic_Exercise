#ifndef ROS_BABEL_FISH_COMPOUND_MESSAGE_HPP
#define ROS_BABEL_FISH_COMPOUND_MESSAGE_HPP

#include "message.hpp"
#include "action_proj/idl/type_support.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>

#include <vector>

namespace ros_babel_fish
{

class CompoundMessage final : public Message
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS( CompoundMessage )

  //! Creates an invalid instance of a compound message
  CompoundMessage();

  CompoundMessage( MessageMembersIntrospection members, std::shared_ptr<void> data );

  explicit CompoundMessage( MessageMembersIntrospection members,
                            rosidl_runtime_cpp::MessageInitialization init =
                                rosidl_runtime_cpp::MessageInitialization::ALL );

  //! Creates a copy of the CompoundMessage pointing at the same message.
  //! Use clone() to get an independent copy of this message.
  CompoundMessage( const CompoundMessage &other );

  CompoundMessage( CompoundMessage && ) = default;

  ~CompoundMessage() override = default;

  //! Datatype of the message, e.g., geometry_msgs::msg::Pose.
  std::string datatype() const;

  //! Name of the message, e.g., geometry_msgs/msg/Pose.
  std::string name() const;

  bool containsKey( const std::string &key ) const;

  std::vector<std::string> keys() const;

  std::string keyAt( size_t index );

  uint32_t memberCount() const { return members_->member_count_; }

  CompoundMessage &operator=( const Message &other );

  CompoundMessage &operator=( const builtin_interfaces::msg::Time &value );

  CompoundMessage &operator=( const builtin_interfaces::msg::Duration &value );

  CompoundMessage &operator=( const rclcpp::Time &value );

  CompoundMessage &operator=( const rclcpp::Duration &value );

  CompoundMessage &operator=( const CompoundMessage &other );

  CompoundMessage &operator=( CompoundMessage &&other ) noexcept;

  /*!
   * Accesses submessage with the given key.
   * @throws std::out_of_range If key is not in message.
   * @throws std::runtime_error If message was not initialized for key.
   * @param key The name of the member you want to access.
   * @return The value of the member with the given name.
   */
  Message &operator[]( const std::string &key ) override;

  const Message &operator[]( const std::string &key ) const override;

  std::vector<Message::SharedPtr> values();

  std::vector<Message::ConstSharedPtr> values() const;

  std::shared_ptr<const void> type_erased_message() const;

  std::shared_ptr<void> type_erased_message();

  /*!
   * Cast this compound message to a message type.
   * Example:
   * \code{.cpp}
   * using geometry_msgs::msg::Point;
   * Point point = compound.message&lt;Point&gt;();
   * \endcode
   *
   * @tparam T The type of the message to cast to.
   * @throws BabelFishException If the message is not of the target type.
   * @return The message casted to the target type.
   */
  template<typename T>
  std::shared_ptr<T> message()
  {
    if ( rosidl_generator_traits::name<T>() != name() ) {
      throw BabelFishException( "Tried to cast compound message of type " + name() +
                                " to incompatible message type " +
                                rosidl_generator_traits::name<T>() + "!" );
    }
    return std::static_pointer_cast<T>( type_erased_message() );
  }

  //! @copydoc message()
  template<typename T>
  std::shared_ptr<const T> message() const
  {
    if ( rosidl_generator_traits::name<T>() != name() ) {
      throw BabelFishException( "Tried to cast compound message of type " + name() +
                                " to incompatible message type " +
                                rosidl_generator_traits::name<T>() + "!" );
    }
    return std::static_pointer_cast<const T>( type_erased_message() );
  }

  //! Creates a copy of this compound message
  CompoundMessage clone() const;

  bool isValid() const;

protected:
  void onMoved() override;

  void initValues() const;

  Message::SharedPtr operator[]( size_t index );

  Message::ConstSharedPtr operator[]( size_t index ) const;

  bool _isMessageEqual( const Message &other ) const override;

  void _assign( const Message &other ) override;

  MessageMembersIntrospection members_;
  mutable std::vector<Message::SharedPtr> values_;
  mutable bool initialized_values_ = false;
};
} // namespace ros_babel_fish

#endif // ROS_BABEL_FISH_COMPOUND_MESSAGE_HPP
