#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <stdlib.h>
#include <chrono>
#include <iostream>

#include <aruco_detection.h>
#include "xquad_autonomous_stack/Aruco.h"
#include "xquad_autonomous_stack/ArucoArray.h"
#include "rtabmap_ros/Point2f.h"

using namespace std;
using namespace Eigen;

ArucoDetection *Ar_Detection;
cv::Mat image;
cv::Mat intrinsic_matrix;
cv::Mat distortion_coeff;

ros::Publisher aruco_pub;
ArucoDetection::Aruco_pose aruco_data;

void proc_aruco(cv::Mat &frame, cv::Mat &Mt, cv::Mat &Ds)
{
	aruco_data = Ar_Detection->Pose_Estimation(frame, Mt, Ds);
	if (aruco_data.isPose)
	{
		xquad_autonomous_stack::ArucoArray aruco_msg;
		xquad_autonomous_stack::Aruco data;
		rtabmap_ros::Point2f corners_aux;
		for (int i = 0; i < aruco_data.id.size(); i++)
		{
			data.id = aruco_data.id[i];
			for (int k = 0; k < 4; k++)
			{
				corners_aux.x = aruco_data.corners[i][k].x;
				corners_aux.y = aruco_data.corners[i][k].y;
				data.corners.push_back(corners_aux);
			}
			cout << data.id << endl;
			aruco_msg.aruco.push_back(data);
			data.corners.clear();
		}
		aruco_pub.publish(aruco_msg);
	}
	else
	{
		xquad_autonomous_stack::ArucoArray aruco_msg;
		ROS_INFO_THROTTLE(5, "There is no pose aruco!");
		aruco_pub.publish(aruco_msg);
	}
}

//-----------------Callback Image Topic------------------
//-------------------------------------------------------
void callback_img(const sensor_msgs::ImageConstPtr &msg)
{
	// ROS_INFO_THROTTLE(1, "Streaming Image.");
	auto start = std::chrono::high_resolution_clock::now();
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	image = cv_ptr->image;
	if (!image.empty())
	{
		proc_aruco(image, intrinsic_matrix, distortion_coeff);
		auto end = std::chrono::high_resolution_clock::now();
		auto dur = end - start;
		auto i_millis = std::chrono::duration_cast<std::chrono::microseconds>(dur);
	}
}
//-------------------------------------------------------
//-------------------------------------------------------


//-----------------Callback Image Topic------------------
//-------------------------------------------------------
void callback_camInfo(const sensor_msgs::CameraInfo &msg)
{

	intrinsic_matrix = (cv::Mat_<float>(3, 3) << msg.K[0], msg.K[1], msg.K[2], msg.K[3], msg.K[4], msg.K[5], msg.K[6], msg.K[7], msg.K[8]);
	distortion_coeff = (cv::Mat_<float>(1, 4) << msg.D[0], msg.D[1], msg.D[2], msg.D[3], msg.D[4]);

}
//-------------------------------------------------------
//-------------------------------------------------------

int main(int argc, char **argv)
{
	Ar_Detection = new ArucoDetection();
	ros::init(argc, argv, "aruco");
	ros::NodeHandle AR;
	aruco_pub = AR.advertise<xquad_autonomous_stack::ArucoArray>("aruco", 1);
	ROS_INFO("Starting ARUCO processing!");

	// FlightGoggles Setup
	intrinsic_matrix = (cv::Mat_<float>(3, 3) << 342.7555236816406, 0.0, 320.0, 0.0, 342.7555236816406, 240.0, 0.0, 0.0, 1.0);
	distortion_coeff = (cv::Mat_<float>(1, 4) << 0.0, 0.0, 0.0, 0.0, 0.0);
	ros::Subscriber info_subscriber = AR.subscribe("/uav/camera/left/camera_info", 1, callback_camInfo);
	ros::Subscriber img_subscriber = AR.subscribe("/uav/camera/left/image_rect_color", 1, callback_img);

	ros::spin();
	return 0;
}
