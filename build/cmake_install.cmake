# Install script for directory: /catkin_ws/Implicit-SDF-Planner/src

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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/catkin_ws/Implicit-SDF-Planner/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/catkin_ws/Implicit-SDF-Planner/install" TYPE PROGRAM FILES "/catkin_ws/Implicit-SDF-Planner/build/catkin_generated/installspace/_setup_util.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/catkin_ws/Implicit-SDF-Planner/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/catkin_ws/Implicit-SDF-Planner/install" TYPE PROGRAM FILES "/catkin_ws/Implicit-SDF-Planner/build/catkin_generated/installspace/env.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/catkin_ws/Implicit-SDF-Planner/install/setup.bash;/catkin_ws/Implicit-SDF-Planner/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/catkin_ws/Implicit-SDF-Planner/install" TYPE FILE FILES
    "/catkin_ws/Implicit-SDF-Planner/build/catkin_generated/installspace/setup.bash"
    "/catkin_ws/Implicit-SDF-Planner/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/catkin_ws/Implicit-SDF-Planner/install/setup.sh;/catkin_ws/Implicit-SDF-Planner/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/catkin_ws/Implicit-SDF-Planner/install" TYPE FILE FILES
    "/catkin_ws/Implicit-SDF-Planner/build/catkin_generated/installspace/setup.sh"
    "/catkin_ws/Implicit-SDF-Planner/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/catkin_ws/Implicit-SDF-Planner/install/setup.zsh;/catkin_ws/Implicit-SDF-Planner/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/catkin_ws/Implicit-SDF-Planner/install" TYPE FILE FILES
    "/catkin_ws/Implicit-SDF-Planner/build/catkin_generated/installspace/setup.zsh"
    "/catkin_ws/Implicit-SDF-Planner/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/catkin_ws/Implicit-SDF-Planner/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/catkin_ws/Implicit-SDF-Planner/install" TYPE FILE FILES "/catkin_ws/Implicit-SDF-Planner/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/catkin_ws/Implicit-SDF-Planner/build/gtest/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/common/quadrotor_msgs/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/debug_assistant/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/uav_simulator/fake_drone/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/common/pose_utils/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/sdf_vis/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/common/manual_take_over/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/common/odom_visualization/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/uav_simulator/local_sensing/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/uav_simulator/mockamap/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/uav_simulator/so3_control/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/common/traj_utils/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/common/uav_utils/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/common/assign_goals/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/common/random_goals/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/uav_simulator/so3_quadrotor_simulator/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/common/rviz_plugins/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/common/selected_points_publisher/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/utils/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/map_manager/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/swept_volume/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/planner_algorithm/cmake_install.cmake")
  include("/catkin_ws/Implicit-SDF-Planner/build/plan_manager/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/catkin_ws/Implicit-SDF-Planner/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
