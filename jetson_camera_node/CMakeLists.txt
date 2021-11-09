cmake_minimum_required(VERSION 2.8.3)
project(jetson_camera_node)

# DOES NOT ACTUALLY CHANGE THE VERSION OF COMPILER, SEE: https://answers.ros.org/question/305153/what-is-adding-stdgnu11-to-my-compiler-options-in-ros-melodic-using-catkin_tools/
#add_compile_options(-std=gnu++17)

# BUT THIS COMMAND WORKS
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

include_directories(.)
include_directories( ${catkin_INCLUDE_DIRS})
add_service_files(
   FILES
   pcSubscribe.srv
   PointCloud.srv
)


generate_messages(
   DEPENDENCIES
)
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jetson_camera_node
  CATKIN_DEPENDS rospy roscpp message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(.)

find_package(realsense2)
find_package(OpenCV 4.5.4 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories( ${catkin_INCLUDE_DIRS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(main_node 
	src/ur10/config.txt
        src/ur10/configOLD.txt
        src/ur10/joint3.stl
        src/ur10/ur10_joint0.STL
        src/ur10/ur10_joint1.STL
        src/ur10/ur10_joint2.STL
        src/ur10/ur10_joint3.STL
        src/ur10/ur10_joint4.STL
        src/ur10/ur10_joint5.STL
        src/ur10/ur10_joint6.STL
        src/ur10_low/ur10_joint0.stl
        src/ur10_low/ur10_joint1.stl
        src/ur10_low/ur10_joint2.stl
        src/ur10_low/ur10_joint3.stl
        src/ur10_low/ur10_joint4.stl
        src/ur10_low/ur10_joint5.stl
        src/ur10_low/ur10_joint6.stl
        src/ur10/workspace2.STL
        src/Aruco.cpp
        src/Aruco.h
        src/Camera.cpp
        src/Camera.h
        src/Clock.h
        src/MainNode.cpp
        src/Engine.cpp
        src/Engine.h
        src/Filter.h
        src/Image.h
        src/Math.h
        src/Object.cpp
        src/Object.h
        src/OctoMap.h
        src/RoboMathSpec.cpp
        src/RoboMathSpec.h
        src/Robot.cpp
        src/Robot.h
        src/RSCamera.cpp
        src/RSCamera.h
        src/Translator.h
        src/_udp.h
        src/_udp.cpp
        src/UdpServer.h
        src/UdpServer.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(main_node ${catkin_LIBRARIES})
target_link_libraries(main_node
        ${OpenCV_LIBS}
        ${realsense2_LIBRARY})


