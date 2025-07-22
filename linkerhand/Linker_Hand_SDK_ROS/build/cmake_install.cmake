# Install script for directory: /home/ros/Linker_Hand_SDK_ROS/src

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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ros/Linker_Hand_SDK_ROS/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ros/Linker_Hand_SDK_ROS/install" TYPE PROGRAM FILES "/home/ros/Linker_Hand_SDK_ROS/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ros/Linker_Hand_SDK_ROS/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ros/Linker_Hand_SDK_ROS/install" TYPE PROGRAM FILES "/home/ros/Linker_Hand_SDK_ROS/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ros/Linker_Hand_SDK_ROS/install/setup.bash;/home/ros/Linker_Hand_SDK_ROS/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ros/Linker_Hand_SDK_ROS/install" TYPE FILE FILES
    "/home/ros/Linker_Hand_SDK_ROS/build/catkin_generated/installspace/setup.bash"
    "/home/ros/Linker_Hand_SDK_ROS/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ros/Linker_Hand_SDK_ROS/install/setup.sh;/home/ros/Linker_Hand_SDK_ROS/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ros/Linker_Hand_SDK_ROS/install" TYPE FILE FILES
    "/home/ros/Linker_Hand_SDK_ROS/build/catkin_generated/installspace/setup.sh"
    "/home/ros/Linker_Hand_SDK_ROS/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ros/Linker_Hand_SDK_ROS/install/setup.zsh;/home/ros/Linker_Hand_SDK_ROS/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ros/Linker_Hand_SDK_ROS/install" TYPE FILE FILES
    "/home/ros/Linker_Hand_SDK_ROS/build/catkin_generated/installspace/setup.zsh"
    "/home/ros/Linker_Hand_SDK_ROS/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ros/Linker_Hand_SDK_ROS/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ros/Linker_Hand_SDK_ROS/install" TYPE FILE FILES "/home/ros/Linker_Hand_SDK_ROS/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ros/Linker_Hand_SDK_ROS/build/gtest/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/L20/L20_get_linker_hand_state/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/L25/be_manipulated/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/finger_guessing/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/get_linker_hand_current/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/get_linker_hand_fault/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/get_linker_hand_force/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/get_linker_hand_speed/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/graphic_display_status/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/gui_control/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/linker_L25_pybullet/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/linker_hand_mujoco/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/linker_hand_pybullet/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/linker_hand_sdk_ros/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/range_to_arc/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/set_linker_hand_current/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/set_linker_hand_speed/cmake_install.cmake")
  include("/home/ros/Linker_Hand_SDK_ROS/build/linker_hand_sdk/examples/set_linker_hand_torque/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/ros/Linker_Hand_SDK_ROS/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
