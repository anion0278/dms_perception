cmake_minimum_required(VERSION 3.10)
project(DMS)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")


include_directories(.)
include_directories(ur10)
include_directories(ur10_low)

find_package(realsense2)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

set(SOURCE_FILES DMS.cpp)

add_executable(DMS
        ur10/config.txt
        ur10/configOLD.txt
        ur10/joint3.stl
        ur10/ur10_joint0.STL
        ur10/ur10_joint1.STL
        ur10/ur10_joint2.STL
        ur10/ur10_joint3.STL
        ur10/ur10_joint4.STL
        ur10/ur10_joint5.STL
        ur10/ur10_joint6.STL
        ur10_low/ur10_joint0.stl
        ur10_low/ur10_joint1.stl
        ur10_low/ur10_joint2.stl
        ur10_low/ur10_joint3.stl
        ur10_low/ur10_joint4.stl
        ur10_low/ur10_joint5.stl
        ur10_low/ur10_joint6.stl
        ur10/workspace2.STL
        Aruco.cpp
        Aruco.h
        Camera.cpp
        Camera.h
        Clock.h
        DMS.cpp
        Engine.cpp
        Engine.h
        Filter.h
        Image.h
        Math.h
        Object.cpp
        Object.h
        OctoMap.h
        RoboMathSpec.cpp
        RoboMathSpec.h
        Robot.cpp
        Robot.h
        RSCamera.cpp
        RSCamera.h
        Translator.h
        _udp.h
        _udp.cpp
        UdpServer.h
        UdpServer.cpp)
target_link_libraries(DMS
        ${OpenCV_LIBS}
        ${realsense2_LIBRARY})
