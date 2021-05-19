/*
 * YoloObjectDetector.cpp
 *
 */

// darknet
#include "YoloObjectDetector.h"
#include <opencv2/opencv.hpp>
#include <string>

// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// Cv-Eigen
#include <opencv2/core/eigen.hpp>

namespace xquad {

YoloObjectDetector::YoloObjectDetector(std::string cfg_filename, std::string weight_filename, int gpu_id):
cfg_filename_(cfg_filename),
weight_filename_(weight_filename)
{
  std::cout << cfg_filename << std::endl;
  std::cout << weight_filename << std::endl;


  detector_ = new Detector(cfg_filename, weight_filename, gpu_id);
  thresh_hold_ = 0.1;
  use_mean_ = false;
}

YoloObjectDetector::~YoloObjectDetector()
{
}



bool YoloObjectDetector::isInitialized(){
	return initialized_;
}


std::vector<bbox_t> YoloObjectDetector::detect(cv::Mat& cv_img, int timestamp){
  std::vector<bbox_t> out_boxes;
  out_boxes = detector_->detect(cv_img, thresh_hold_, use_mean_);
	return out_boxes;
}

// VICTOR
void YoloObjectDetector::poseEstimation(Eigen::VectorXd &pose,std::vector<cv::Point2f>& image_points, std::vector<cv::Point3f>& object_points){
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);

  cv::Mat cam(3, 3, cv::DataType<double>::type);
  cv::Mat distort(4, 1, cv::DataType<double>::type);

  cam = cameraMatrix_;
  distort = distCoeffs_;


  //VectorXd pose(7);

  //cv::solvePnP(object_points, image_points, cam, distort, rvec, tvec);
  cv::solvePnPRansac(object_points, image_points, cam, distort, rvec, tvec);

  cv::Mat rotation_mat(3, 3, cv::DataType<double>::type);
  cv::Rodrigues(rvec, rotation_mat);

  //tranlation
  pose(0) = tvec.at<double>(0);
  pose(1) = tvec.at<double>(1);
  pose(2) = tvec.at<double>(2);

  Eigen::Matrix3d mat;
  cv::cv2eigen(rotation_mat, mat);


  //rotation

  Eigen::Quaterniond EigenQuat(mat);

  pose(3) = EigenQuat.w();
  pose(4) = EigenQuat.x();
  pose(5) = EigenQuat.y();
  pose(6) = EigenQuat.z();


  return;
}


/**/
/*


void YoloObjectDetector::poseEstimation(geometry_msgs::Pose &pose, std::vector<cv::Point2f>& image_points, std::vector<cv::Point3f>& object_points, cv::Mat &camera_matrix, cv::Mat& dist_coeffs){
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);

  cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

  cv::Mat rotation_mat(3, 3, cv::DataType<double>::type);
  cv::Rodrigues(rvec, rotation_mat);

  //tranlation
  pose.position.x = tvec.at<double>(0);
  pose.position.y = tvec.at<double>(1);
  pose.position.z = tvec.at<double>(2);

  tf::Matrix3x3 rosmat(rotation_mat.at<double>(0,0), rotation_mat.at<double>(0,1), rotation_mat.at<double>(0,2),
                       rotation_mat.at<double>(1,0), rotation_mat.at<double>(1,1), rotation_mat.at<double>(1,2),
                       rotation_mat.at<double>(2,0), rotation_mat.at<double>(2,1), rotation_mat.at<double>(2,2));
  //rotation

  tf::Quaternion q;
  rosmat.getRotation(q);
  pose.orientation.x = q[0];
  pose.orientation.y = q[1];
  pose.orientation.z = q[2];
  pose.orientation.w = q[3];

  return;
}




*/
} /* namespace darknet_ros*/
