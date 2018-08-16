cmake_minimum_required(VERSION 2.8.3)
project(wheel_odom)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    message_generation
    )

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   steeringAngle.msg
   wheelSpeeds.msg
)
generate_messages(
    DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
    nav_msgs
)
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES wheel_odom
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(
    ${catkin_INCLUDE_DIRS}
    "include")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

set(wheel_odom_LIB_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/WheelOdom2/WheelOdom2.cpp
    ${PROJECT_SOURCE_DIR}/src/WheelOdometryROS/WheelOdometryROS.cpp
    )

add_executable(wheel_odom
    ${wheel_odom_LIB_SOURCE_FILES}
    src/wheelOdomNode.cpp )
target_link_libraries(wheel_odom    dw     ${catkin_LIBRARIES}  )

add_executable(wheel_pub    src/wheel_pub.cpp )
target_link_libraries(wheel_pub    dw     ${catkin_LIBRARIES}  )


