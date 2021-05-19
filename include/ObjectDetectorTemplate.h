/*
 * YoloObjectDetectorTemplate.h
 *
 * XQUAD TEAM 
 */

#pragma once
// darknet 
// #include "darknet/utils.h"
#include "darknet/yolo_v2_class.hpp"

#include <set>
#include <opencv2/opencv.hpp>


namespace xquad {

class ObjectDetectorTemplate
{
 public:
  virtual std::vector<bbox_t> detect(cv::Mat &image, int timestamp) = 0;
};


} /*xquad namespace*/
