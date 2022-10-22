#include <iostream>
#include <ximea_cv/ximea_cv.hpp>
#include <ximea_cv/ximea_ros.hpp>
#include <signal.h>

XimeaROS::XimeaROS(int argc, char** argv) {
    ros::init(argc, argv, "ximea_camera");
	exposure_time = 8000.0;
	gain = 0.0;
}

/**
 * @brief Initialise Camera Publishers
 * 
 * @param serials List of serial numbers to initialise
 */
void XimeaROS::init_camera_pub(std::vector<std::string> serials) {
    // For each device, create node handles and publishers.
    for (std::string serial : serials) {

        nh_camera_info[serial]	= std::make_unique<ros::NodeHandle>();
        nh_camera[serial]		= std::make_unique<ros::NodeHandle>(std::string("ximea_ros/ximea_") + serial);
        pub_camera_info[serial]	= nh_camera_info[serial]->advertise<sensor_msgs::CameraInfo>(std::string("ximea_ros/ximea_") + serial + std::string("/camera_info"), 1);
        camera_info_manager[serial]	= std::make_unique<camera_info_manager::CameraInfoManager>(*nh_camera[serial], std::string("ximea_") + serial);

        if (!camera_info_manager[serial]->loadCameraInfo("")) {
            std::cout << "Failed to get calibration for camera: " << serial << std::endl;
        }

        camera_info[serial] = camera_info_manager[serial]->getCameraInfo();
    }
}

void XimeaROS::init_img_pub(std::vector<std::string> serials) {
    for (std::string serial : serials) {

        it_img[serial]   = std::make_unique<image_transport::ImageTransport>(ros::NodeHandle()); // Image Transport Node Handle for Color Frame
        
        pub_img[serial]  = it_img[serial]->advertise(std::string("ximea_ros/ximea_") + serial + "/image_raw", 1);               // Publisher for Color Frame
    }
}


void XimeaROS::send_image(std::string serial, cv::Mat img, std::string format) {
    // Create ROS sensor messages from OpenCV Images
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), format, img).toImageMsg();

    // Publish the sensor messages
    pub_img[serial].publish(msg);

    // Create and publish the camera info
    sensor_msgs::CameraInfo info_camera = camera_info_manager[serial]->getCameraInfo();
    ros::Time now = ros::Time::now();
    info_camera.header.frame_id = std::string("ximea_") + serial;
    info_camera.header.stamp = now;
    pub_camera_info[serial].publish(info_camera);
}
