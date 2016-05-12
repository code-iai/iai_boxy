cmake_minimum_required(VERSION 2.8.3)
project(iai_boxy_teleop)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(boxy_teleop_joy src/${PROJECT_NAME}/boxy_joy.cpp)
add_dependencies(boxy_teleop_joy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(boxy_teleop_joy ${catkin_LIBRARIES})
