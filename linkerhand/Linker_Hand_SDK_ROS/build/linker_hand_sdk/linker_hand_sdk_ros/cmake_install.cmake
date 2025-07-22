# Install script for directory: /home/ros/Linker_Hand_SDK_ROS/src/linker_hand_sdk/linker_hand_sdk_ros

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ros/Linker_Hand_SDK_ROS/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/linker_hand_sdk_ros/catkin_generated/installspace/linker_hand_sdk_ros.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/linker_hand_sdk_ros/cmake" TYPE FILE FILES
    "/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/linker_hand_sdk_ros/catkin_generated/installspace/linker_hand_sdk_rosConfig.cmake"
    "/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/linker_hand_sdk_ros/catkin_generated/installspace/linker_hand_sdk_rosConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/linker_hand_sdk_ros" TYPE FILE FILES "/home/ros/Linker_Hand_SDK_ROS/src/linker_hand_sdk/linker_hand_sdk_ros/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/linker_hand_sdk_ros" TYPE PROGRAM FILES "/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/linker_hand_sdk_ros/catkin_generated/installspace/linker_hand.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/linker_hand_sdk_ros" TYPE PROGRAM FILES "/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/linker_hand_sdk_ros/catkin_generated/installspace/linker_hand_l25.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/linker_hand_sdk_ros" TYPE PROGRAM FILES "/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/linker_hand_sdk_ros/catkin_generated/installspace/linker_hand_l7.py")
endif()

