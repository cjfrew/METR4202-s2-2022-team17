cmake_minimum_required( VERSION 3.16.3 )

set(ximea_color_VERSION 1.0)

project( ximea_color )


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
  LIBRARIES ximea_cv
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime message_runtime ximea_ros
)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

  
add_executable( ximea_color_detect src/ximea_color_detect.cpp )

target_link_libraries( ximea_color_detect ${ximea_ROS_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES} m3api )

install(TARGETS ximea_color_detect
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/example_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
