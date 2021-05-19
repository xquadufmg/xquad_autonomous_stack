// ROS
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>

// ARUCO MSG
#include "xquad_autonomous_stack/Aruco.h"
#include "xquad_autonomous_stack/ArucoArray.h"
#include "rtabmap_ros/Point2f.h"
#include <aruco_detection.h>

// OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <mutex>
#include <tuple>

// EIGEN
#include <eigen3/Eigen/Dense>

// YOLO INCLUDES
#include <YoloObjectDetector.h>
#include <GateProcessing.h>

using namespace Eigen;
using namespace std;


#define NONTRACKED 9000
xquad::YoloObjectDetector *yolo;
GateProcessing gate_identification(3.3528, 4); // gate size and number of gates;
vector<Gate_t> identificated_gates;
int current_curve = 0;
std::vector<bbox_t> gates;
int num_detected_gates = 0;
std::vector<cv::Rect2d> bboxes;
bool tracking_ok = false;
VectorXd pose(7);
VectorXd pose_raw(7);
cv::Mat image;
cv::Mat intrinsic_matrix;
cv::Mat distortion_coeff;
ArucoDetection *Ar_Detection;
ArucoDetection::Aruco_pose aruco_data;

cv::Mat printed_image;

std::mutex camera_mutex_;
std::mutex aruco_mutex_;



//-----------------Callback Aruco Detection---------------
//-------------------------------------------------------
void callback_aruco(const xquad_autonomous_stack::ArucoArray& msg){
	ArucoDetection::Aruco_pose aruco_data_received;
	for(int i=0; i<msg.aruco.size(); i++){
		aruco_data_received.id.push_back(msg.aruco[i].id);
		std::vector<cv::Point2f> corner_aux;
		for(int k=0; k<4; k++){
			cv::Point2f point_aux;
			point_aux.x = msg.aruco[i].corners[k].x;
			point_aux.y = msg.aruco[i].corners[k].y;
			corner_aux.push_back(point_aux);
		}	
		aruco_data_received.corners.push_back(corner_aux);	
	}
	aruco_mutex_.lock();
	aruco_data = aruco_data_received;
	aruco_mutex_.unlock();
	aruco_data_received.id.clear();
	aruco_data_received.corners.clear();
}
//-------------------------------------------------------
//-------------------------------------------------------



//-----------------Callback Image Topic------------------
//-------------------------------------------------------
void callback_img(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    camera_mutex_.lock();
    image = cv_ptr->image;
    camera_mutex_.unlock();
}
//-------------------------------------------------------
//-------------------------------------------------------



Eigen::MatrixXd getGateParametes(ros::NodeHandle nh, int id){
  MatrixXd temp_m(4,3);

  XmlRpc::XmlRpcValue my_list;
  if (id == 8)
  {
    id = 54;
  }
  nh.getParam("/uav/Gate"+std::to_string(id+1)+"/nominal_location", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  temp_m << my_list[0][0], my_list[0][1], my_list[0][2],
            my_list[1][0], my_list[1][1], my_list[1][2],
            my_list[2][0], my_list[2][1], my_list[2][2],
            my_list[3][0], my_list[3][1], my_list[3][2];

  return temp_m;
}



//---------------- YOLO JOB -----------------------------
//-------------------------------------------------------
VectorXd angles_pos(3);
std::vector<int> idvec(2, 0);
Vector3d position_ant = Vector3d::Zero();
//Flag for initialization
bool flag_posOk = false;

// void proc_yolo_detect(ros::NodeHandle nh){
xquad_autonomous_stack::ArucoArray proc_yolo_detect(ros::NodeHandle nh){
	xquad_autonomous_stack::ArucoArray gate_corners;

    if (image.empty()){ // just check if the image isn't empty
		cout<<"Camera image is empety!\n";
    }
    else{

	    // Yolo Searching Gates
	    std::vector<bbox_t> gate_local;
	    gate_local = yolo->detect(image, 80);
	    // cout << "YOLO OK \n";


	    // Get Aruco information
	    aruco_mutex_.lock();
	    std::vector<int> id_(aruco_data.id);
	    std::vector<std::vector<cv::Point2f>> corners_(aruco_data.corners);
	    aruco_mutex_.unlock();

	    // Gate association to aruco and define corners
	    std::vector<bbox_t> Corners_square;
	    gate_identification.identify(corners_, id_, image, gate_local, identificated_gates, idvec, Corners_square, current_curve);


	    // GATE Get Pose
	    std::vector<xquad_autonomous_stack::ArucoArray> all_gate_corners;
	    tie(printed_image,all_gate_corners) = gate_identification.getPose(image, identificated_gates, intrinsic_matrix, distortion_coeff);
	    
		if(all_gate_corners.size() > 0){
			gate_corners = all_gate_corners[0];
		}
		
    }

    return gate_corners;


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



void callback_curve(const std_msgs::Int32ConstPtr &msg)
{

	current_curve = msg->data;
}
//-------------------------------------------------------
//-------------------------------------------------------



int main(int argc, char **argv){
   
    // Define Camera Parameters
	intrinsic_matrix = (cv::Mat_<float>(3, 3) << 342.7555236816406, 0.0, 320.0, 0.0, 342.7555236816406, 240.0, 0.0, 0.0, 1.0);
	distortion_coeff = (cv::Mat_<float>(1, 4) << 0.0, 0.0, 0.0, 0.0, 0.0);
    // Initialization of Aruco Class
    Ar_Detection = new ArucoDetection();


    // Create Ros Node
    ros::init(argc, argv, "yolo_nh");
    ros::NodeHandle yolo_nh("~");

    XmlRpc::XmlRpcValue init_pose;
    yolo_nh.getParam("/uav/flightgoggles_uav_dynamics/init_pose", init_pose);
    pose << double(init_pose[0]),double(init_pose[1]),double(init_pose[2]),double(init_pose[6]),double(init_pose[3]),double(init_pose[4]),double(init_pose[5]);
    pose_raw << 0,0,0,0,0,0,0;

    // Initialization of Yolo
    std::string yolo_tiny_cfg, yolo_tiny_weights;
	yolo_nh.param<std::string>("yolo_tiny_cfg", yolo_tiny_cfg, "yolov4-tiny.cfg");
	yolo_nh.param<std::string>("yolo_tiny_weights", yolo_tiny_weights, "yolov4-tiny_best.weights");

	// Initialization of Yolo
	ROS_INFO("[Perception] Loading yolo tiny cfg file: %s", yolo_tiny_cfg.c_str());
	ROS_INFO("[Perception] Loading yolo tiny weights file: %s", yolo_tiny_weights.c_str());
	ROS_INFO("[Perception] Starting Yolo-Tiny");
	yolo = new xquad::YoloObjectDetector(yolo_tiny_cfg.c_str(), yolo_tiny_weights.c_str());


    // Define publisher
    ros::Publisher pub_pose = yolo_nh.advertise<geometry_msgs::PoseStamped>("/pose_beacons", 1);
    ros::Publisher pub_pose_raw = yolo_nh.advertise<geometry_msgs::PoseStamped>("/pose_beacons_raw", 1);
    ros::Publisher pub_corners = yolo_nh.advertise<xquad_autonomous_stack::ArucoArray>("/gate/corners_point", 1);

    // Define Subscribers
	std::string camera_topic;
	yolo_nh.param<std::string>("camera_topic", camera_topic, "/uav/camera/left/image_rect_color");
	ros::Subscriber info_subscriber = yolo_nh.subscribe("/uav/camera/left/camera_info", 1, callback_camInfo);
	ros::Subscriber img_subscriber = yolo_nh.subscribe(camera_topic.c_str(), 1, callback_img);
	ros::Subscriber curve_subscriber = yolo_nh.subscribe("/path/curve_now", 1, callback_curve);
    ros::Subscriber aruco_subscriber = yolo_nh.subscribe("/aruco", 1, callback_aruco);


    ROS_INFO("[Perception] Starting ROS Loop-Control");
	double loop_frequency;
	yolo_nh.param<double>("loop_frequency", loop_frequency, 10.0);
	ROS_INFO("[Perception] Setting Loop-Control to %f Hz. ", loop_frequency);
	ros::Rate loop_rate(loop_frequency);

    ros::spinOnce();

    while (ros::ok()){
    	xquad_autonomous_stack::ArucoArray corner_msgs;
		corner_msgs = proc_yolo_detect(yolo_nh);
		pub_corners.publish(corner_msgs);		

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
