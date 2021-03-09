# Install script for directory: /Users/ad1t7a/Developer/adi/third_party/rbdl

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
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Library/Developer/CommandLineTools/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/ad1t7a/Developer/adi/third_party/rbdl/build/librbdl.2.6.0.dylib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.2.6.0.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.2.6.0.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Library/Developer/CommandLineTools/usr/bin/strip" -x "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.2.6.0.dylib")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/ad1t7a/Developer/adi/third_party/rbdl/build/librbdl.dylib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Library/Developer/CommandLineTools/usr/bin/strip" -x "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.dylib")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rbdl" TYPE FILE FILES
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/build/include/rbdl/rbdl_config.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/Body.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/Constraints.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/Dynamics.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/Joint.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/Kinematics.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/Logging.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/Model.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/Quaternion.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/SpatialAlgebraOperators.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/compileassert.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/rbdl.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/rbdl_eigenmath.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/rbdl_math.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/rbdl_mathutils.h"
    "/Users/ad1t7a/Developer/adi/third_party/rbdl/include/rbdl/rbdl_utils.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/Users/ad1t7a/Developer/adi/third_party/rbdl/build/rbdl.pc")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/ad1t7a/Developer/adi/third_party/rbdl/build/addons/urdfreader/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/Users/ad1t7a/Developer/adi/third_party/rbdl/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
