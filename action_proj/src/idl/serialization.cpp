#include "action_proj/idl/serialization.hpp"
#include "action_proj/macros.hpp"
#include "action_proj/messages/compound_message.hpp"
#include "action_proj/messages/value_message.hpp"

namespace ros_babel_fish
{
std::shared_ptr<void>
createContainer( const rosidl_typesupport_introspection_cpp::MessageMembers &members,
                 rosidl_runtime_cpp::MessageInitialization initialization )
{
  auto result = std::shared_ptr<void>( new unsigned char[members.size_of_], [members]( void *data ) {
    members.fini_function( data );
    delete[] static_cast<unsigned char *>( data );
  } );
  members.init_function( result.get(), initialization );
  return result;
}

std::shared_ptr<void> createContainer( const MessageMembersIntrospection &members,
                                       rosidl_runtime_cpp::MessageInitialization initialization )
{
  return createContainer( *members.value, initialization );
}
} // namespace ros_babel_fish
