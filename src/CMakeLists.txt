cmake_minimum_required(VERSION 2.8)
#project(telemetry)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")


if(NOT ONBOARDSDK_SOURCE)
    set(ONBOARDSDK_SOURCE "/home/flight/lib/dji/osdk-core")
endif()




if(NOT IO_SOURCE)
    set(IO_SOURCE "/home/flight/lib/io/src")
endif()


if(NOT MODULES_HEADER_DIR)
    set(MODULES_HEADER_DIR "${ONBOARDSDK_SOURCE}/modules/inc/payload")
endif()

if(NOT FLIGHT_MODULES_HEADER_DIR)
    set(FLIGHT_MODULES_HEADER_DIR "${ONBOARDSDK_SOURCE}/modules/inc/flight")
endif()




include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ONBOARDSDK_SOURCE}/protocol/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/linux/inc)
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/core/inc)
include_directories(${MODULES_HEADER_DIR})
include_directories(${FLIGHT_MODULES_HEADER_DIR})
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)
include_directories(${IO_SOURCE})
FILE(GLOB SOURCE_FILES *.hpp *.cpp
        #${CMAKE_CURRENT_SOURCE_DIR}/common/dji_linux_environment.cpp
        #${CMAKE_CURRENT_SOURCE_DIR}/common/dji_linux_helpers.cpp
        )
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} djiosdk-core)
target_link_libraries(${PROJECT_NAME} sioclient)
target_link_libraries(${PROJECT_NAME} pthread)

message(STATUS ${Boost_INCLUDE_DIRS} )
#target_include_directories(sio_console_demo PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../src" ${Boost_INCLUDE_DIRS} )
