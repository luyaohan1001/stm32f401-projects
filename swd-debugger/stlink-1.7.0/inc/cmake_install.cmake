# Install script for directory: /home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/inc

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/stlink" TYPE FILE FILES
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/inc/backend.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/inc/stlink.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/inc/stm32.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/inc/version.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/chipid.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/commands.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/flash_loader.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/helper.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/libusb_settings.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/logging.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/md5.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/reg.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/sg.h"
    "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/src/stlink-lib/usb.h"
    )
endif()

