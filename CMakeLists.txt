cmake_minimum_required(VERSION 2.8.3)
project(treasure_hunting)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  mavros
  mavros_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV)

include_directories(
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

LINK_DIRECTORIES("usr/local/lib")


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   mavros_msgs
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES treasure_hunting
   CATKIN_DEPENDS mavros mavros_msgs roscpp rospy std_msgs cv_bridge
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(treasure_hunting src/mavrosCommandMain.cpp src/mavrosCommand.cpp)
target_link_libraries(treasure_hunting ${catkin_LIBRARIES} ${OpenCV_LIBS} serial)

