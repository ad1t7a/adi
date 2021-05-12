cmake_minimum_required(VERSION 3.2)

set(TARGET UrDriver)

# UrDriver
#--------------------------------------------------
set(${TARGET}_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/ur_driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/ur_realtime_communication.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/robot_state.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/robot_state_RT.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/do_output.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/ur_communication.cpp
  )

set(${TARGET}_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/Inc/ur_modern_driver/do_output.h
  ${CMAKE_CURRENT_SOURCE_DIR}/Inc/ur_modern_driver/robot_state.h
  ${CMAKE_CURRENT_SOURCE_DIR}/Inc/ur_modern_driver/robot_state_RT.h
  ${CMAKE_CURRENT_SOURCE_DIR}/Inc/ur_modern_driver/ur_communication.h
  ${CMAKE_CURRENT_SOURCE_DIR}/Inc/ur_modern_driver/ur_driver.h
  ${CMAKE_CURRENT_SOURCE_DIR}/Inc/ur_modern_driver/ur_realtime_communication.h
 )

if ( DEFINED ENV{QNX_HOST} )
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif ( DEFINED ENV{QNX_HOST} )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Inc)
add_library(${TARGET} ${${TARGET}_SRC} ${${TARGET}_HDR})


