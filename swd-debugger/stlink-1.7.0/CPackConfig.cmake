# This file will be configured to contain variables for CPack. These variables
# should be set in the CMake list file of the project before CPack module is
# included. The list of available CPACK_xxx variables and their associated
# documentation may be obtained using
#  cpack --help-variable-list
#
# Some variables are common to all generators (e.g. CPACK_PACKAGE_NAME)
# and some are specific to a generator
# (e.g. CPACK_NSIS_EXTRA_INSTALL_COMMANDS). The generator specific variables
# usually begin with CPACK_<GENNAME>_xxxx.


set(CPACK_BUILD_SOURCE_DIRS "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0;/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0")
set(CPACK_CMAKE_GENERATOR "Unix Makefiles")
set(CPACK_COMPONENT_UNSPECIFIED_HIDDEN "TRUE")
set(CPACK_COMPONENT_UNSPECIFIED_REQUIRED "TRUE")
set(CPACK_DEBIAN_FILE_NAME "DEB-DEFAULT")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/cmake/packaging/deb/changelog;/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/cmake/packaging/deb/copyright;/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/cmake/packaging/deb/rules;/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/cmake/packaging/deb/postinst")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "pkg-config, build-essential, debhelper (>=9), cmake (>= 3.4.2), libusb-1.0-0-dev (>= 1.0.20)")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Nightwalker-87 <stlink-org>")
set(CPACK_DEBIAN_PACKAGE_RELEASE "1")
set(CPACK_DEBIAN_PACKAGE_SUGGESTS "libgtk-3-dev, pandoc")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_FILE "/usr/share/cmake-3.20/Templates/CPack.GenericDescription.txt")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_SUMMARY "stlink built using CMake")
set(CPACK_GENERATOR "DEB;RPM")
set(CPACK_INSTALL_CMAKE_PROJECTS "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0;stlink;ALL;/")
set(CPACK_INSTALL_PREFIX "/usr/local")
set(CPACK_MODULE_PATH "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/cmake/modules")
set(CPACK_NSIS_DISPLAY_NAME "stlink 1.7.0")
set(CPACK_NSIS_INSTALLER_ICON_CODE "")
set(CPACK_NSIS_INSTALLER_MUI_ICON_CODE "")
set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
set(CPACK_NSIS_PACKAGE_NAME "stlink 1.7.0")
set(CPACK_NSIS_UNINSTALL_NAME "Uninstall")
set(CPACK_OUTPUT_CONFIG_FILE "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CPackConfig.cmake")
set(CPACK_OUTPUT_FILE_PREFIX "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/dist")
set(CPACK_PACKAGE_DEFAULT_LOCATION "/")
set(CPACK_PACKAGE_DESCRIPTION "Open source STM32 MCU programming toolset")
set(CPACK_PACKAGE_DESCRIPTION_FILE "/usr/share/cmake-3.20/Templates/CPack.GenericDescription.txt")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "stlink built using CMake")
set(CPACK_PACKAGE_FILE_NAME "stlink-1.7.0-Linux")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/stlink-org/stlink")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "stlink 1.7.0")
set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "stlink 1.7.0")
set(CPACK_PACKAGE_NAME "stlink")
set(CPACK_PACKAGE_RELOCATABLE "true")
set(CPACK_PACKAGE_VENDOR "stlink-org")
set(CPACK_PACKAGE_VERSION "1.7.0")
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "1")
set(CPACK_PACKAGE_VERSION_PATCH "1")
set(CPACK_RESOURCE_FILE_LICENSE "/usr/share/cmake-3.20/Templates/CPack.GenericLicense.txt")
set(CPACK_RESOURCE_FILE_README "/usr/share/cmake-3.20/Templates/CPack.GenericDescription.txt")
set(CPACK_RESOURCE_FILE_WELCOME "/usr/share/cmake-3.20/Templates/CPack.GenericWelcome.txt")
set(CPACK_RPM_CHANGELOG_FILE "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/cmake/packaging/rpm/changelog")
set(CPACK_RPM_FILE_NAME "RPM-DEFAULT")
set(CPACK_RPM_PACKAGE_DESCRIPTION "CPACK_DEBIAN_PACKAGE_DESCRIPTION")
set(CPACK_RPM_PACKAGE_LICENSE "BSD-3")
set(CPACK_RPM_PACKAGE_RELEASE "1")
set(CPACK_SET_DESTDIR "OFF")
set(CPACK_SOURCE_GENERATOR "TBZ2;TGZ;TXZ;TZ")
set(CPACK_SOURCE_OUTPUT_CONFIG_FILE "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CPackSourceConfig.cmake")
set(CPACK_SOURCE_RPM "OFF")
set(CPACK_SOURCE_TBZ2 "ON")
set(CPACK_SOURCE_TGZ "ON")
set(CPACK_SOURCE_TXZ "ON")
set(CPACK_SOURCE_TZ "ON")
set(CPACK_SOURCE_ZIP "OFF")
set(CPACK_SYSTEM_NAME "Linux")
set(CPACK_THREADS "1")
set(CPACK_TOPLEVEL_TAG "Linux")
set(CPACK_WIX_SIZEOF_VOID_P "8")

if(NOT CPACK_PROPERTIES_FILE)
  set(CPACK_PROPERTIES_FILE "/home/luyaohan1001/Projects/stm32f401-projects/swd-debugger/stlink-1.7.0/CPackProperties.cmake")
endif()

if(EXISTS ${CPACK_PROPERTIES_FILE})
  include(${CPACK_PROPERTIES_FILE})
endif()
