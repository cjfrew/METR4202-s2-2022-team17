cmake_minimum_required( VERSION 3.16.3 )

set(ximea_ros_VERSION 1.0)

project( ximea_ros )


find_package( OpenCV REQUIRED )
find_package( catkin REQUIRED COMPONENTS
  roscpp
  rospy
	pcl_conversions
	pcl_ros
	std_msgs
	sensor_msgs
	image_transport
	cv_bridge
	camera_info_manager
	pcl_conversions
)
find_package( PkgConfig REQUIRED )

if (catkin_FOUND)
  message(STATUS "Found.")
else()
  message(STATUS "Not found.")
endif()


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime message_runtime
)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
 
  
add_library( ximea_cv src/ximea_cv.cpp )
add_library( ximea_ros src/ximea_ros.cpp )
add_executable( ximea_demo src/ximea_demo.cpp )

target_link_libraries( ximea_demo ximea_cv ximea_ros ${OpenCV_LIBS} ${catkin_LIBRARIES} m3api )
target_link_libraries( ximea_ros ximea_cv ${OpenCV_LIBS} ${catkin_LIBRARIES} m3api )
target_link_libraries( ximea_cv ${OpenCV_LIBS} ${catkin_LIBRARIES} m3api )


install(TARGETS ximea_demo ximea_cv ximea_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
