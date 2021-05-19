#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;

class ArucoDetection{
  private:
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;
    cv::Ptr<cv::aruco::Dictionary>  dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    double W, H;
    cv::Mat intrinsic_matrix;
    cv::Mat distortion_coeff;
    std::vector<cv::Point3f> aruco_3d_points;
    cv::Mat image;

    VectorXd pose;

  public:
    ArucoDetection();
    ~ArucoDetection();
    // VectorXd Pose_Estimation(cv::Mat, cv::Mat, cv::Mat);
    struct Aruco_pose{
      bool isPose;
      std::vector<VectorXd> pose;
      std::vector<int> id;
      std::vector<std::vector<cv::Point2f>> corners;
    };
    Aruco_pose Pose_Estimation(cv::Mat, cv::Mat, cv::Mat);

//    void setVectors(cv::Mat rot_vec, cv::Mat transl_vec);

//    cv::Mat rvec;
//	cv::Mat tvec;


};
