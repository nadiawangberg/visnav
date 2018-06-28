cmake_minimum_required(VERSION 2.8.3)
project(visnav)

add_definitions(-std=c++11 -Wall -Werror)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	sensor_msgs
	geometry_msgs
	mavros_msgs
	cv_bridge
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES visnav
	CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs cv_bridge
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )


set(VisNav_Ros_SRC
	
	include/visnav/ColorManager.h
	include/visnav/PixelPoint.h
	include/visnav/ImageLoader.h
	include/visnav/Navigation.h
	include/visnav/OpticalFlowSparse.h
	include/visnav/Visualizer.h

	src/ColorManager.cpp
	src/PixelPoint.cpp
	src/ImageLoader.cpp
	src/Navigation.cpp
	src/OpticalFlowSparse.cpp
	src/Visualizer.cpp
)



source_group(VisNav FILES ${VisNav_Ros_SRC})

add_library(${PROJECT_NAME} ${VisNav_Ros_SRC})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(opticalflow_node src/opticalflow_node.cpp)
target_link_libraries(opticalflow_node ${PROJECT_NAME})

add_executable(imagesave_node src/imagesave_node.cpp)
target_link_libraries(imagesave_node ${PROJECT_NAME})

add_executable(pose_visualisation_node src/pose_visualisation_node.cpp)
target_link_libraries(pose_visualisation_node ${PROJECT_NAME})

add_executable(orb_slam_node src/ros_mono.cpp)
target_link_libraries(orb_slam_node ${PROJECT_NAME})



find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

