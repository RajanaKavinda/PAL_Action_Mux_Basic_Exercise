cmake_minimum_required(VERSION 3.8)
project(action_proj)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(action_proj_interfaces REQUIRED)


set(SOURCES
  src/detail/babel_fish_subscription.cpp
  src/idl/providers/local_type_support_provider.cpp
  src/idl/serialization.cpp
  src/idl/type_support_provider.cpp
  src/messages/array_message.cpp
  src/messages/compound_message.cpp
  src/messages/message.cpp
  src/babel_fish.cpp
  src/detail/topic.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_compile_definitions(${PROJECT_NAME} PUBLIC RCLCPP_VERSION_MAJOR=${rclcpp_VERSION_MAJOR})
target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
ament_target_dependencies(${PROJECT_NAME} ament_index_cpp rclcpp rclcpp_action rcpputils rosidl_runtime_cpp rosidl_typesupport_cpp rosidl_typesupport_introspection_cpp)

add_executable(action_client scripts/action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action std_msgs action_proj_interfaces)

add_executable(action_server scripts/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action action_proj_interfaces)

add_executable(generic_subscriber scripts/generic_subscriber.cpp)
target_link_libraries(generic_subscriber  ${PROJECT_NAME})
target_include_directories(generic_subscriber  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(generic_subscriber geometry_msgs)

add_executable(publisher scripts/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(string_publisher scripts/string_publisher.cpp)
ament_target_dependencies(string_publisher rclcpp std_msgs)

add_executable(int32_publisher scripts/int32_publisher.cpp)
ament_target_dependencies(int32_publisher  rclcpp std_msgs)

## Mark libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Mark scripts for installation
install(TARGETS
  action_server
  action_client
  generic_subscriber
  string_publisher
  int32_publisher
  publisher
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action rosidl_runtime_cpp rosidl_typesupport_cpp rosidl_typesupport_introspection_cpp)

ament_package()
