#ifndef XIMEA_ROS_HPP
#define XIMEA_ROS_HPP

#include <m3api/xiApi.h>
#include <stdio.h>
#include <opencv2/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <stdint.h>
#include <vector>
#include <map>


#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>

class XimeaROS {
private:
    std::map<std::string, std::unique_ptr<ros::NodeHandle>> nh_camera_info;
    std::map<std::string, std::unique_ptr<ros::NodeHandle>> nh_camera;
    std::map<std::string, std::unique_ptr<image_transport::ImageTransport>> it_img;
    std::map<std::string, ros::Publisher> pub_camera_info;
    std::map<std::string, image_transport::Publisher> pub_img;
    std::map<std::string, std::unique_ptr<camera_info_manager::CameraInfoManager>> camera_info_manager;
    std::map<std::string, sensor_msgs::CameraInfo> camera_info;
public:
    XimeaROS(int argc, char** argv);
    void init_camera_pub(std::vector<std::string> serials);
    void init_img_pub(std::vector<std::string> serials);
    void send_image(std::string serial, cv::Mat img, std::string format);
	float exposure_time;
	float gain;
};

#endif