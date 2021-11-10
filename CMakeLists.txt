cmake_minimum_required(VERSION 3.16.0)


project(drone_stocktaker VERSION 0.1.0)



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
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  
)






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

