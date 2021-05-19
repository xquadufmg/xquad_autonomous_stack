#pragma once

#include "ros/ros.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <vector>
#include <cmath>
#include <tuple>

#include <eigen3/Eigen/Dense>

#include "YoloObjectDetector.h"

#include "libcbdetect/boards_from_cornres.h"
#include "libcbdetect/config.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/plot_boards.h"
#include "libcbdetect/plot_corners.h"

#include "xquad_autonomous_stack/Aruco.h"
#include "rtabmap_ros/Point2f.h"
#include "xquad_autonomous_stack/ArucoArray.h"

//#include "config.h"

#define UNKNOW_GATE_ID -1

using namespace std;
using namespace cv;

struct Section_State_t
{
    double dist_cam_to_gate;
    int gate_id;
};

struct Gate_t
{
    unsigned int x, y, w, h;
    float prob;
    int gate_id;
    Rect rect_gate;

    Point corner_1;
    Rect rect_1;
    float prob_1;

    Point corner_2;
    Rect rect_2;
    float prob_2;

    Point corner_3;
    Rect rect_3;
    float prob_3;

    Point corner_4;
    Rect rect_4;
    float prob_4;

    vector<Point2f> corners;

    cv::Mat rvec;
    cv::Mat tvec;

    bool has_pose;
    std::vector<int> corners_founded;
};

struct Aruco_Rect_
{
    int id;
    Point center;
    Rect rect;
};

class GateProcessing
{
private:
    vector<Gate_t> identificated_gates_last;
    std::vector<std::vector<cv::Point3f>> corners_points;
    std::vector<cv::Point3f> corner_points;
    std::vector<cv::Point3d> gate_points_d;
    std::vector<Section_State_t> state_gate_id;
    int section_state;
    std::vector<int> idvec;
    int number_of_gates;

public:
    GateProcessing(double scalar, int number_of_gates);
    ~GateProcessing();
    void identify(std::vector<std::vector<cv::Point2f>> corners, std::vector<int> id, Mat frame,
                  std::vector<bbox_t> gates_yolo, vector<Gate_t> &identificated_gates);
    void identify(std::vector<std::vector<cv::Point2f>> markerCorners, std::vector<int> markerIds, Mat frame, std::vector<bbox_t> gates_yolo, vector<Gate_t> &identificated_gates, std::vector<int> &idvec, std::vector<bbox_t> &Corners_square, int current_curve);
    std::tuple< cv::Mat, std::vector<xquad_autonomous_stack::ArucoArray> > getPose(cv::Mat frame, vector<Gate_t> &identificated_gates, cv::Mat intrinsic_matrix, cv::Mat distortion_coeff);
    Rect enlargeROI(Mat frm, Rect boundingBox, int padding);
};
