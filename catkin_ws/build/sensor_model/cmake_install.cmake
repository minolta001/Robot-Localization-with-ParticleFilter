# Install script for directory: /home/lichen_ubuntu_t480/OneDrive_minolta001@gmail.com/UmassAmherst/2023_Spring/COMP_603/P3/Robot-Localization-with-ParticleFilter/catkin_ws/src/sensor_model

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lichen_ubuntu_t480/OneDrive_minolta001@gmail.com/UmassAmherst/2023_Spring/COMP_603/P3/Robot-Localization-with-ParticleFilter/catkin_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lichen_ubuntu_t480/OneDrive_minolta001@gmail.com/UmassAmherst/2023_Spring/COMP_603/P3/Robot-Localization-with-ParticleFilter/catkin_ws/build/sensor_model/catkin_generated/installspace/sensor_model.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_model/cmake" TYPE FILE FILES
    "/home/lichen_ubuntu_t480/OneDrive_minolta001@gmail.com/UmassAmherst/2023_Spring/COMP_603/P3/Robot-Localization-with-ParticleFilter/catkin_ws/build/sensor_model/catkin_generated/installspace/sensor_modelConfig.cmake"
    "/home/lichen_ubuntu_t480/OneDrive_minolta001@gmail.com/UmassAmherst/2023_Spring/COMP_603/P3/Robot-Localization-with-ParticleFilter/catkin_ws/build/sensor_model/catkin_generated/installspace/sensor_modelConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sensor_model" TYPE FILE FILES "/home/lichen_ubuntu_t480/OneDrive_minolta001@gmail.com/UmassAmherst/2023_Spring/COMP_603/P3/Robot-Localization-with-ParticleFilter/catkin_ws/src/sensor_model/package.xml")
endif()

