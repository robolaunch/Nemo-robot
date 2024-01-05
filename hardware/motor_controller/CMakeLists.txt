cmake_minimum_required(VERSION 3.8)
project(motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(robolaunch_msgs REQUIRED)
# find_package(ros2_port_finder REQUIRED)
find_package(message_filters REQUIRED)
find_package(serial REQUIRED)

include_directories(include ${BOOST_INCLUDE_DIRS})

add_executable(motor_controller src/motor_controller.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES})
ament_target_dependencies(motor_controller rclcpp robolaunch_msgs serial)

install(TARGETS
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
