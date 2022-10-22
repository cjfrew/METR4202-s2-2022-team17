#include <iostream>
#include <ximea_cv/ximea_cv.hpp>
#include <ximea_cv/ximea_ros.hpp>
#include <signal.h>

bool rgb_toggle = false;

void toggle_callback(std_msgs::Bool msg) {
	rgb_toggle = msg.data;
}


int main(int argc, char** argv) {
	std::vector<std::string> formats = {
		"mono8",
		"bgr8"
	};
	std::vector<XI_IMG_FORMAT> img_formats = {
		XI_RAW8,
		XI_RGB24
	};
	XimeaROS ximea_ros(argc, argv);
	
	std::vector<std::string> serials = {
		"32702151"
	};

    ros::NodeHandle nh_rgb;
	ros::Subscriber sub_rgb;
	XI_IMG_FORMAT img_format = img_formats[0];
	std::string format = formats[0];

	rgb_toggle = false;

	ximea_ros.init_camera_pub(serials);
	ximea_ros.init_img_pub(serials);

	std::map<std::string, std::unique_ptr<xiAPIplusCameraOcv>> cam;
	for (std::string serial : serials) {
		cam[serial] = std::make_unique<xiAPIplusCameraOcv>();
		cam[serial]->OpenBySN(serial.c_str());
	}
	for (std::string serial : serials) {
		cam[serial]->SetExposureTime(ximea_ros.exposure_time);
		cam[serial]->SetGain(ximea_ros.gain);
		cam[serial]->SetDownsampling((XI_DOWNSAMPLING_VALUE)2);
		cam[serial]->SetImageDataFormat(img_format);
	}
	
	for (std::string serial : serials) {
		cam[serial]->StartAcquisition();
	}
	
	nh_rgb = ros::NodeHandle();
	sub_rgb = nh_rgb.subscribe("/ximea_ros/show_rgb", 1000, toggle_callback);

	bool rgb_flag = rgb_toggle;
	while (ros::ok()) {
		for (std::string serial : serials) {


			cv::Mat img = cam[serial]->GetNextImageOcvMat();
			ximea_ros.send_image(serial, img, format);
			cv::imshow(serial, img);
			char c = cv::waitKey(1);

			if (c == '=') {
				ximea_ros.gain += 0.5;
				if (ximea_ros.gain > 15.0) {
					ximea_ros.gain = 15.0;
				}
				cam[serial]->SetGain(ximea_ros.gain);
			}
			if (c == '-') {
				ximea_ros.gain -= 0.5;
				if (ximea_ros.gain < 0.0) {
					ximea_ros.gain = 0.0;
				}
				cam[serial]->SetGain(ximea_ros.gain);
			}
			
			if (c == ' ' ) {
				rgb_toggle = !rgb_toggle;
			}
			if (rgb_flag != rgb_toggle) {
				rgb_flag = rgb_toggle;
				if (!rgb_toggle) {
					img_format = img_formats[0];
					format = formats[0];
				} else {
					img_format = img_formats[1];
					format = formats[1];
				}
			cam[serial]->SetImageDataFormat(img_format);
			}
			cv::waitKey(1);
		}
		ros::spinOnce();
	}
	
	for (std::string serial : serials) {
		cam[serial]->StopAcquisition();
	}

	for (std::string serial : serials) {
		cam[serial]->Close();
	}
	return 0;
}
