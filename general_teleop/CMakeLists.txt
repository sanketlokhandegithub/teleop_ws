cmake_minimum_required(VERSION 3.0.2)
project(general_teleop)

add_compile_options(-std=c++17)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  genmsg
)


## Declare ROS messages and services
 add_message_files(
   FILES
   Num.msg
   LegCommand.msg
   LegCommandArray.msg
   MotorCommand.msg
 )

 add_service_files(
   FILES
   AddTwoInts.srv
 )


## Generate added messages and services
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

## Build talker and listener
include_directories(
   include ${catkin_INCLUDE_DIRS}
)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker general_teleop_generate_messages_cpp)

# add_executable(listener src/listener.cpp)
# target_link_libraries(listener ${catkin_LIBRARIES})
# add_dependencies(listener general_teleop_generate_messages_cpp)


# add_executable(testpos src/test.cpp)
# target_link_libraries(testpos ${catkin_LIBRARIES})
# add_dependencies(testpos general_teleop_generate_messages_cpp)

# add_executable(printpos src/printpos.cpp)
# target_link_libraries(printpos ${catkin_LIBRARIES})
# add_dependencies(printpos general_teleop_generate_messages_cpp)

