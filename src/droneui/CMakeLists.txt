add_subdirectory(test)

add_library(droneui src/droneui.cpp)

target_include_directories(droneui PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)
target_link_libraries(droneui PUBLIC
	${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(droneui PUBLIC imcore ${catkin_LIBRARIES} ${OpenCV_LIBS})



