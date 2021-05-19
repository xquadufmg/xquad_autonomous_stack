/*
 * YoloObjectDetector.h
 *
 * XQUAD TEAM 
 */

#pragma once

#define OPENCV
// c++

// darknet 
#include "ObjectDetectorTemplate.h"
#include "darknet/yolo_v2_class.hpp"

#include <math.h>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


#include <opencv2/opencv.hpp>

namespace xquad {

class YoloObjectDetector: public ObjectDetectorTemplate {
 public:

  YoloObjectDetector(std::string cfg_filename,
  std::string weight_filename, int gpu_id = 0);

  ~YoloObjectDetector();

  bool isInitialized();

  cv::Mat cameraMatrix_;
  cv::Mat distCoeffs_;
  void poseEstimation(Eigen::VectorXd &pose,std::vector<cv::Point2f>& image_points, std::vector<cv::Point3f>& object_points);

  std::vector<bbox_t> detect(cv::Mat &image, int timestamp);

private:
  Detector *detector_;
  bool initialized_;
  std::string cfg_filename_;
  std::string weight_filename_;
  float thresh_hold_;
  bool use_mean_;

};

} /* namespace xquad*/
