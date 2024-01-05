cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robolaunch_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(include)

add_executable(inverse_kinematics src/inverse_kinematics.cpp)
target_link_libraries(inverse_kinematics ${catkin_LIBRARIES})
ament_target_dependencies(inverse_kinematics rclcpp robolaunch_msgs geometry_msgs)

install(TARGETS
        inverse_kinematics
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
