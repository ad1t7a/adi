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


set(CPACK_BUILD_SOURCE_DIRS "/Users/ad1t7a/Developer/adi/third_party/rbdl;/Users/ad1t7a/Developer/adi/third_party/rbdl/build")
set(CPACK_CMAKE_GENERATOR "Unix Makefiles")
set(CPACK_COMPONENT_UNSPECIFIED_HIDDEN "TRUE")
set(CPACK_COMPONENT_UNSPECIFIED_REQUIRED "TRUE")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Martin Felis <martin@fysx.org>")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_FILE "/usr/local/Cellar/cmake/3.19.3/share/cmake/Templates/CPack.GenericDescription.txt")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_SUMMARY "RBDL built using CMake")
set(CPACK_GENERATOR "DEB")
set(CPACK_INSTALL_CMAKE_PROJECTS "/Users/ad1t7a/Developer/adi/third_party/rbdl/build;RBDL;ALL;/")
set(CPACK_INSTALL_PREFIX "/usr/local")
set(CPACK_MODULE_PATH "/Users/ad1t7a/Developer/adi/third_party/rbdl/CMake")
set(CPACK_NSIS_DISPLAY_NAME "CPACK_PACKAGE 2.6")
set(CPACK_NSIS_INSTALLER_ICON_CODE "")
set(CPACK_NSIS_INSTALLER_MUI_ICON_CODE "")
set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
set(CPACK_NSIS_PACKAGE_NAME "CPACK_PACKAGE 2.6")
set(CPACK_NSIS_UNINSTALL_NAME "Uninstall")
set(CPACK_OSX_SYSROOT "/Library/Developer/CommandLineTools/SDKs/MacOSX11.1.sdk")
set(CPACK_OUTPUT_CONFIG_FILE "/Users/ad1t7a/Developer/adi/third_party/rbdl/build/CPackConfig.cmake")
set(CPACK_PACKAGE_DEFAULT_LOCATION "/")
set(CPACK_PACKAGE_DESCRIPTION_FILE "/Users/ad1t7a/Developer/adi/third_party/rbdl/README.md")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "the Rigid Body Dynamics Library (RBDL)")
set(CPACK_PACKAGE_EXECUTABLES "rbdl_luamodel_util;RBDL Lua Model Utility")
set(CPACK_PACKAGE_FILE_NAME "rbdl--2.6.0")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "CPACK_PACKAGE 2.6")
set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "CPACK_PACKAGE 2.6")
set(CPACK_PACKAGE_NAME "RBDL")
set(CPACK_PACKAGE_RELOCATABLE "true")
set(CPACK_PACKAGE_VENDOR "Martin Felis")
set(CPACK_PACKAGE_VERSION "2.6.0")
set(CPACK_PACKAGE_VERSION_MAJOR "2")
set(CPACK_PACKAGE_VERSION_MINOR "6")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_RESOURCE_FILE_LICENSE "/Users/ad1t7a/Developer/adi/third_party/rbdl/LICENSE")
set(CPACK_RESOURCE_FILE_README "/usr/local/Cellar/cmake/3.19.3/share/cmake/Templates/CPack.GenericDescription.txt")
set(CPACK_RESOURCE_FILE_WELCOME "/usr/local/Cellar/cmake/3.19.3/share/cmake/Templates/CPack.GenericWelcome.txt")
set(CPACK_SET_DESTDIR "OFF")
set(CPACK_SOURCE_GENERATOR "TBZ2;TGZ;TXZ;TZ")
set(CPACK_SOURCE_OUTPUT_CONFIG_FILE "/Users/ad1t7a/Developer/adi/third_party/rbdl/build/CPackSourceConfig.cmake")
set(CPACK_SOURCE_RPM "OFF")
set(CPACK_SOURCE_TBZ2 "ON")
set(CPACK_SOURCE_TGZ "ON")
set(CPACK_SOURCE_TXZ "ON")
set(CPACK_SOURCE_TZ "ON")
set(CPACK_SOURCE_ZIP "OFF")
set(CPACK_SYSTEM_NAME "Darwin")
set(CPACK_TOPLEVEL_TAG "Darwin")
set(CPACK_WIX_SIZEOF_VOID_P "8")

if(NOT CPACK_PROPERTIES_FILE)
  set(CPACK_PROPERTIES_FILE "/Users/ad1t7a/Developer/adi/third_party/rbdl/build/CPackProperties.cmake")
endif()

if(EXISTS ${CPACK_PROPERTIES_FILE})
  include(${CPACK_PROPERTIES_FILE})
endif()
