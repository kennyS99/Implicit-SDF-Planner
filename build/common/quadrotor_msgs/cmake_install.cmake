# Install script for directory: /catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/catkin_ws/Implicit-SDF-Planner/install")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/msg" TYPE FILE FILES
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/AuxCommand.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/Corrections.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/Gains.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/OutputData.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/PositionCommand.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/PPROutputData.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/Serial.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/SO3Command.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/StatusData.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/TRPYCommand.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/Odometry.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/PolynomialTrajectory.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/LQRTrajectory.msg"
    "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/msg/GoalSet.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/cmake" TYPE FILE FILES "/catkin_ws/Implicit-SDF-Planner/build/common/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/catkin_ws/Implicit-SDF-Planner/devel/include/quadrotor_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/catkin_ws/Implicit-SDF-Planner/devel/share/roseus/ros/quadrotor_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/catkin_ws/Implicit-SDF-Planner/devel/share/common-lisp/ros/quadrotor_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/catkin_ws/Implicit-SDF-Planner/devel/share/gennodejs/ros/quadrotor_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/catkin_ws/Implicit-SDF-Planner/devel/lib/python3/dist-packages/quadrotor_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/catkin_ws/Implicit-SDF-Planner/devel/lib/python3/dist-packages/quadrotor_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/catkin_ws/Implicit-SDF-Planner/build/common/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/cmake" TYPE FILE FILES "/catkin_ws/Implicit-SDF-Planner/build/common/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs/cmake" TYPE FILE FILES
    "/catkin_ws/Implicit-SDF-Planner/build/common/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgsConfig.cmake"
    "/catkin_ws/Implicit-SDF-Planner/build/common/quadrotor_msgs/catkin_generated/installspace/quadrotor_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/quadrotor_msgs" TYPE FILE FILES "/catkin_ws/Implicit-SDF-Planner/src/common/quadrotor_msgs/package.xml")
endif()

