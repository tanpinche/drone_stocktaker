cmake_minimum_required(VERSION 3.16.0)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Found ccache")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

project(drone_stocktaker VERSION 0.1.0)

set(BUILD_TESTING on)

## Project Options
option(BUILD_TESTING "Build tests" ON)
option(STATIC_CHECK "Perform static check" OFF)

## Check if code compiles on x86_64 platform
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
	set(BUILD_ON_X86_64 TRUE)
else()
	set(BUILD_ON_X86_64 FALSE)
endif()

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(STATIC_CHECK)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        message(STATUS "Found cppcheck")
        set(CMAKE_CXX_CPPCHECK cppcheck;--std=c++11;--enable=all)
    endif() 
endif()

find_package(
  OpenCV REQUIRED
)
find_package(catkin REQUIRED COMPONENTS
  espdrone_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



## Additional cmake module path
# include(${CMAKE_BINARY_DIR}/conan_paths.cmake)
set(USER_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${USER_CMAKE_PATH}/modules")
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")

## Set compiler to use c++ 14 features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Chosse build type


## Use GNUInstallDirs to install libraries into correct locations on all platforms.
include(GNUInstallDirs)


## Add source directory
add_subdirectory(src)

install(TARGETS imcore droneui 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY src/droneui/src/imcore/imgui 
                src/droneui/src/imcore/implot
                src/droneui/src/droneui/include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN "*.h*")

