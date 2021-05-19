// kalman filter
// ROS stuffs
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include <flightgoggles/IRMarkerArray.h>
 
#include <tf/transform_broadcaster.h>

//Timer
#include <chrono>
#include <thread>

//Eigen stuff
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
//File management
#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/stat.h> // commands
#include <stdio.h>

//Our libs
#include "EKF_class.h"

#define PI 3.1415926535

/*
Universidade Federal de Minas Gerais (UFMG) - 2020
XQuad Project
Laboratory: CORO
Contact:
Adriano M. C. Rezende, <adrianomcr18@gmail.com>
Victor R. F. Miranda <victormrfm@gmail.com>
*/


//Uncomment the line bellow in order to use ground truth data in the filter's correction
// #define USE_GT

//Uncomment the line bellow in order to print the states on the screen
// #define PRINT_STATES 1



using namespace std;
using namespace std::this_thread;
using namespace std::chrono;

//Counters for ecimation
int count_imu = 0;
int count_beacons = 0;
int count_slam = 0;

//Flags for new measured data
bool new_u_imu = false; //imu
bool new_z_beacons = false; //beacons - YOLO
bool new_z_beaconsRaw = false; //beacons - YOLO
bool new_z_slam = false; // visual odometry - orbslam
// bool first_beacon = false; // first beacon data
bool first_beacon = true; // first beacon data

//Variables to store new received data
Eigen::VectorXd u_imu(10,1);
Eigen::VectorXd z_beacons(7,1);
Eigen::VectorXd z_beacons_raw(7,1);
Eigen::VectorXd z_slam(7,1);
Eigen::VectorXd gt_data(10,1);

//Variables to transform quaternion <-> Euler
Eigen::VectorXd pos_euler(6,1);
Eigen::VectorXd pos_quat(7,1);

//Flag for initialization
bool filter_init = false;

// Int curve now
int curve_now = 0;
Eigen::MatrixXd gate_world_beacons(4,3);


// #ifdef USE_GT
void GT_callback(const tf2_msgs::TFMessage::ConstPtr &msg){

  if(msg->transforms[0].child_frame_id != "uav/imu") return;

  // filter_init = true;

  // Estimate time elapsed since last measurement
  geometry_msgs::TransformStamped uav_tf = msg->transforms[0];
  static ros::Time t_anterior = uav_tf.header.stamp;
  ros::Duration dt = uav_tf.header.stamp - t_anterior;
  t_anterior = uav_tf.header.stamp ;
  double dt_sec = dt.toSec();
  if(dt_sec == 0.0 )
    dt_sec = 1/120.0;



  // Compute velocity
  gt_data.block(7,0,3,1) << (msg->transforms[0].transform.translation.x - gt_data(0))/dt_sec, (msg->transforms[0].transform.translation.y - gt_data(1))/dt_sec, (msg->transforms[0].transform.translation.z - gt_data(2))/dt_sec;

  //Get position
  gt_data.block(0,0,3,1) << msg->transforms[0].transform.translation.x, msg->transforms[0].transform.translation.y, msg->transforms[0].transform.translation.z;

  //Get orientation
  gt_data.block(3,0,4,1) << msg->transforms[0].transform.rotation.w, msg->transforms[0].transform.rotation.x, msg->transforms[0].transform.rotation.y, msg->transforms[0].transform.rotation.z;

  // new_z_beacons = true;
}
// #endif



// callbacks for IMU data
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){

  u_imu << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
         1*msg->angular_velocity.x, 1*msg->angular_velocity.y, 1*msg->angular_velocity.z,
         msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;

  new_u_imu = true;

}


void init_callback(const flightgoggles::IRMarkerArray::ConstPtr& msg){
  if(first_beacon != true){
    first_beacon = true;
  }

}


void curve_callback(const std_msgs::Int32::ConstPtr& msg){
  curve_now = msg->data;
}


void beacons_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  Eigen::VectorXd new_beacons(7,1);
  double alpha = 0.3;

  //TEMPORARY!!!!!!!!!!!!!!!
  if(!filter_init){
    filter_init = true;
    cout << "\33[92mFilter initialized\33[0m" << endl;
  }

  //Get position
  // new_beacons.block(0,0,3,1) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  z_beacons.block(0,0,3,1) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  //Get orientation
  // new_beacons.block(3,0,4,1) << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
  z_beacons.block(3,0,4,1) << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;


  // z_beacons = (1-alpha)*z_beacons + alpha*new_beacons;
  new_z_beacons = true;
  
}

void beacons_raw_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  Eigen::VectorXd new_beacons(7,1);
  double alpha = 0.3;

  //TEMPORARY!!!!!!!!!!!!!!!
  if(!filter_init){
    filter_init = true;
    cout << "\33[92mFilter initialized\33[0m" << endl;
  }

  //Get position
  // new_beacons.block(0,0,3,1) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  z_beacons_raw.block(0,0,3,1) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  //Get orientation
  // new_beacons.block(3,0,4,1) << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
  z_beacons_raw.block(3,0,4,1) << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;


  // z_beacons = (1-alpha)*z_beacons + alpha*new_beacons;
  new_z_beaconsRaw = true;
  
}






void slam_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

  if(!filter_init){
    filter_init = true;
    cout << "\33[92mFilter initialized\33[0m" << endl;
  }

  //Get position
  z_slam.block(0,0,3,1) << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  //Get orientation
  z_slam.block(3,0,4,1) << msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;

  new_z_slam = true;

}








// Rotation Matrix body to inertial
Eigen::MatrixXd R_bw(const Eigen::VectorXd &u){

  // matriz de rotacao
  double e_w, e_x, e_y, e_z;
  e_w = u[0]; e_x = u[1]; e_y = u[2]; e_z = u[3];

  // Create a rotation matrix from body- to inertial-frame
  Eigen::MatrixXd R_bw(3,3); // Rotation matrix body-frame to navigation-frame
  R_bw << pow(e_x,2)+pow(e_w,2)-pow(e_y,2)-pow(e_z,2), -2*e_z*e_w+2*e_y*e_x, 2*e_y*e_w +2*e_z*e_x,
  2*e_x*e_y+ 2*e_w*e_z, pow(e_y,2)+pow(e_w,2)-pow(e_x,2)-pow(e_z,2), 2*e_z*e_y-2*e_x*e_w,
  2*e_x*e_z-2*e_w*e_y, 2*e_y*e_z+2*e_w*e_x, pow(e_z,2)+pow(e_w,2)-pow(e_x,2)-pow(e_y,2);

  return R_bw;
}




#define N_STATES 21
bool enable_slam;
bool enable_txt_log;
Eigen::VectorXd EKF_states_0(N_STATES);
Eigen::MatrixXd EKF_H(3,N_STATES);
Eigen::MatrixXd EKF_H_yolo(6,N_STATES);
Eigen::MatrixXd EKF_H_slam(6,N_STATES);
Eigen::MatrixXd EKF_Q(N_STATES,N_STATES);
Eigen::MatrixXd EKF_Q_bar(6,6);
Eigen::MatrixXd EKF_R(3,3);
Eigen::MatrixXd EKF_R_yolo(6,6);
Eigen::MatrixXd EKF_R_slam(6,6);
Eigen::MatrixXd EKF_P(N_STATES,N_STATES);



void load_EKF_parameters(ros::NodeHandle nh){

  std::vector<double> temp_vector;

  try{

    nh.getParam("/EKF/enable_slam", enable_slam);

    nh.getParam("/EKF/enable_txt_log", enable_txt_log);

    nh.getParam("/EKF/states_0", temp_vector);
    EKF_states_0 = VectorXd::Map(temp_vector.data(), temp_vector.size());

    nh.getParam("/EKF/H", temp_vector);
    EKF_H = Eigen::Map<Eigen::Matrix<double, N_STATES, 3> >(temp_vector.data()).transpose();
    nh.getParam("/EKF/H_yolo", temp_vector);
    EKF_H_yolo = Eigen::Map<Eigen::Matrix<double, N_STATES, 6> >(temp_vector.data()).transpose();
    nh.getParam("/EKF/H_slam", temp_vector);
    EKF_H_slam = Eigen::Map<Eigen::Matrix<double, N_STATES, 6> >(temp_vector.data()).transpose();

    nh.getParam("/EKF/Q", temp_vector);
    EKF_Q = Eigen::Map<Eigen::Matrix<double, N_STATES, N_STATES> >(temp_vector.data()).transpose();

    nh.getParam("/EKF/Q_bar", temp_vector);
    EKF_Q_bar = Eigen::Map<Eigen::Matrix<double, 6, 6> >(temp_vector.data()).transpose();

    nh.getParam("/EKF/R", temp_vector);
    EKF_R = Eigen::Map<Eigen::Matrix<double, 3, 3> >(temp_vector.data()).transpose();
    nh.getParam("/EKF/R_yolo", temp_vector);
    EKF_R_yolo = Eigen::Map<Eigen::Matrix<double, 6, 6> >(temp_vector.data()).transpose();
    nh.getParam("/EKF/R_slam", temp_vector);
    EKF_R_slam = Eigen::Map<Eigen::Matrix<double, 6, 6> >(temp_vector.data()).transpose();

    nh.getParam("/EKF/P", temp_vector);
    EKF_P = Eigen::Map<Eigen::Matrix<double, N_STATES, N_STATES> >(temp_vector.data()).transpose();


    printf("\33[92m\nThe following EKF parameters were loaded:\33[0m\n\n");
    cout << "\33[92menable_slam:\n" << enable_slam << "\33[0m" << endl << endl;
    cout << "\33[92menable_txt_log:\n" << enable_txt_log << "\33[0m" << endl << endl;
    cout << "\33[92mEKF_states_0:\n" << EKF_states_0 << "\33[0m" << endl << endl;
    cout << "\33[92mEKF_H:\n" << EKF_H << "\33[0m" << endl << endl;
    cout << "\33[92mEKF_H_yolo:\n" << EKF_H_yolo << "\33[0m" << endl << endl;
    cout << "\33[92mEKF_H_slam:\n" << EKF_H_slam << "\33[0m" << endl << endl;
    cout << "\33[92mEKF_Q:\n" << EKF_Q << "\33[0m" << endl << endl;
    cout << "\33[92mEKF_Q_bar:\n" << EKF_Q_bar << "\33[0m" << endl << endl;
    cout << "\33[92mEKF_R:\n" << EKF_R << "\33[0m" << endl << endl;
    cout << "\33[92mEKF_R_yolo:\n" << EKF_R_yolo << "\33[0m" << endl << endl;
    cout << "\33[92mEKF_R_slam:\n" << EKF_R_slam << "\33[0m" << endl << endl;
    cout << "\33[92mEKF_P:\n" << EKF_P << "\33[0m" << endl << endl;

  }catch(...){

    printf("\33[41mError when trying to read EKF parameters\33[0m");

  }



}


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



int main(int argc, char *argv[]){

  //Initialize node
  ros::init(argc, argv, "filter");

  // Get handle
  ros::NodeHandle nh_;
  // Handle for getting parameters
  ros::NodeHandle nh_2("filter");

  // Callbacks
  ros::Subscriber sub_imu = nh_.subscribe("/uav/sensors/imu", 1, &imu_callback);
// #ifdef USE_GT
  ros::Subscriber sub_GT = nh_.subscribe("/tf", 1, &GT_callback);
// #else
  ros::Subscriber sub_beacons = nh_.subscribe("/pose_beacons", 1, &beacons_callback);
  ros::Subscriber sub_beacons_raw = nh_.subscribe("/pose_beacons_raw", 1, &beacons_raw_callback);  
// #endif  
  ros::Subscriber sub_curve = nh_.subscribe("/path/curve_now", 1, &curve_callback);

  // ros::Subscriber sub_slam = nh_.subscribe("/orb_slam_2/pose", 1, &slam_callback);
  ros::Subscriber sub_slam = nh_.subscribe("/orbslam2/new_pose", 1, &slam_callback);
  ros::Subscriber sub_init = nh_.subscribe("/uav/camera/left/ir_beacons", 1, &init_callback);

  //Publishers
  ros::Publisher states_pub = nh_.advertise<std_msgs::Float32MultiArray>("/states_filter", 50);
  ros::Publisher odom_pub = nh_.advertise<nav_msgs::Odometry>("/ekf/odom", 1);
  ros::Publisher marker_pub = nh_.advertise<visualization_msgs::Marker>("/visualization_marker_states", 1);
  ros::Publisher arm_pub = nh_.advertise<std_msgs::Empty>("/uav/input/arm", 1);
  ros::Publisher pub_start = nh_.advertise<std_msgs::Bool>("/flag_start",1);


  //Used messages
  nav_msgs::Odometry drone_odom;
  geometry_msgs::Pose drone_pose;
  visualization_msgs::Marker drone_marker;

  // Initialize more variables
  double dt = 0.0;
  double t_current = ros::Time::now().toSec();
  double t_previous = ros::Time::now().toSec();
  double t_init = ros::Time::now().toSec();
  double time_log = 0.0;
  int imu_step = 1; // decimate imu data
  int beacons_step = 1; // decimate beacons ddata
  int slam_step = 1; // decimate slam data

  Eigen::Matrix3d R;
  Eigen::Matrix3d R_;
  Eigen::Vector3d velo_w;


  //Read parameters of the EKF filter
  load_EKF_parameters(nh_2);

  // EKF filter object
  double frequency = 300.0;
  EKF_class *Filter;
  Filter = new EKF_class(EKF_states_0, frequency, enable_slam, EKF_H, EKF_H_yolo, EKF_H_slam, EKF_Q, EKF_Q_bar, EKF_R, EKF_R_yolo, EKF_R_slam, EKF_P);
  Eigen::VectorXd states(N_STATES);
  Eigen::VectorXd states_bias(6);
  //Local states variable
  states.setZero();
  states_bias.setZero();


  //Define frequency
  ros::Rate loop_rate(frequency*1.05);

  // ================================ Create log =================================
  FILE *state_log;
  FILE *imu_log;
  FILE *beacons_log;
  FILE *slam_log;
  FILE *GT_log;
  FILE *beaconsRaw_log;

  if(enable_txt_log){
    std::string log_path;
    std::string log_path_1;
    std::string log_path_2;
    std::string log_path_3;
    std::string log_path_4;
    std::string log_path_5;
    std::string log_path_6;
    if (nh_2.getParam ("/EKF/log_path", log_path)){
      cout << "\33[92mSetting log path from parameter\33[0m" << endl;
      log_path_1 = log_path+"state_log.txt";
      log_path_2 = log_path+"imu_log.txt";
      log_path_3 = log_path+"beacons_log.txt";
      log_path_4 = log_path+"slam_log.txt";
      log_path_5 = log_path+"GT_log.txt";
      log_path_6 = log_path+"beaconsRaw_log.txt";
    }
    try{
      state_log = fopen(log_path_1.c_str(),"w");
      imu_log = fopen(log_path_2.c_str(),"w");
      beacons_log = fopen(log_path_3.c_str(),"w");
      slam_log = fopen(log_path_4.c_str(),"w");
      GT_log = fopen(log_path_5.c_str(),"w");
      beaconsRaw_log = fopen(log_path_6.c_str(),"w");
    } catch(...){
      cout << "\33[41mError when oppening the log files\33[0m" << endl;
    }

    if(!enable_txt_log){
      fprintf(imu_log,"%f\n",0.0); //clear the file if nothing will be writtten to it
    }

  }




  //Main loop
  while (ros::ok()){
    static bool flag_initialize = false;
    
    if(first_beacon == true and flag_initialize == false){
      std_msgs::Empty msg_arm;
      std_msgs::Bool bool_msg;
      bool_msg.data = true;
      pub_start.publish(bool_msg);
      sleep_for(nanoseconds(1000000000));
      pub_start.publish(bool_msg);
      sleep_for(nanoseconds(1000000000));
      pub_start.publish(bool_msg);
      sleep_for(nanoseconds(5000000000));
      arm_pub.publish(msg_arm);
      flag_initialize = true;
    }


    // ==================================================
    // Perform a prediction step with the IMU information
    // ==================================================
    if(new_u_imu == true){
      new_u_imu = false;

      //Compute dt
      t_current = ros::Time::now().toSec();
      dt = t_current - t_previous;

      t_previous = t_current;

      if (filter_init==true){


        //Update the IMU data in the filter object
        double Ts = dt;
        Filter->callback_imu(u_imu.block(3,0,3,1), u_imu.block(0,0,3,1)); //(gyro,accel)

        //Perform a prediction step in the filter
        Filter->prediction(Ts);

        // Get the current states of the filter
        states = Filter->get_states();

        // Get the current bias
        states_bias = Filter->get_bias();


#ifdef PRINT_STATES
        Eigen::VectorXd states_raw(21);
        states_raw = Filter->get_raw_states();
        cout << "\33[40mpos:" << states_raw.block(0,0,3,1).transpose() << "\33[0m" << endl;
        cout << "\33[99mrpy:" << states_raw.block(3,0,3,1).transpose() << "\33[0m" << endl;
        cout << "\33[40mvel:" << states_raw.block(6,0,3,1).transpose() << "\33[0m" << endl;
        cout << "\33[99mbias_gyro:" << states_raw.block(9,0,3,1).transpose() << "\33[0m" << endl;
        cout << "\33[40mbias_acc:" << states_raw.block(12,0,3,1).transpose() << "\33[0m" << endl;
        cout << "\33[99mbias_slam_pos:" << states_raw.block(15,0,3,1).transpose() << "\33[0m" << endl;
        cout << "\33[40mbias_slam_rpy:" << states_raw.block(18,0,3,1).transpose() << "\33[0m" << endl;
        cout << "\33[99mvel_GT:" << gt_data.block(6,0,3,1).transpose() << "\33[0m" << endl;
        cout << "\33[40mvel:" << velo_w.transpose() << "\33[0m" << endl;
        cout << endl;
#endif


        //Save data to log files
        if(enable_txt_log){
          
          time_log = ros::Time::now().toSec() - t_init;

          //Save imu data
          fprintf(imu_log,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",u_imu(0),u_imu(1),u_imu(2),u_imu(3),u_imu(4),u_imu(5),u_imu(6),u_imu(7),u_imu(8),u_imu(9),time_log);
          fflush(imu_log);

          //Save states
          fprintf(state_log,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",states(0),states(1),states(2),states(3),states(4),states(5),states(6),states(7),states(8),states(9),time_log);
          // fprintf(state_log,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",states(0),states(1),states(2),states(3),states(4),states(5),states(6),states(7),states(8),states(9),states_bias(0),states_bias(1),states_bias(2),states_bias(3),states_bias(4),states_bias(5),time_log);
          fflush(state_log);

          //Save GT
          fprintf(GT_log,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",gt_data(0),gt_data(1),gt_data(2),gt_data(3),gt_data(4),gt_data(5),gt_data(6),time_log);
          fflush(GT_log);
        }
      }
    }


    // ===========================================
    // Update filter with information from beacons
    // ===========================================
    // if(new_z_beacons == true){
    //   new_z_beacons = false;
    if(new_z_beaconsRaw == true){
      new_z_beaconsRaw = false;
      count_beacons++;

      if (count_beacons == beacons_step){
        count_beacons = 0;

        if (filter_init==true){

          //Call update with raw pose
          pos_euler.block(0,0,3,1) = z_beacons_raw.block(0,0,3,1);
          pos_euler.block(3,0,3,1) = Filter->quat2eulerangle(z_beacons_raw.block(3,0,4,1));
          gate_world_beacons = getGateParametes(nh_2,curve_now);
          bool flag_pub = false;
          flag_pub = Filter->callback_pose_yolo4(pos_euler, EKF_R_yolo, gate_world_beacons);

          //Save data to log files
          if(enable_txt_log && flag_pub){

            time_log = ros::Time::now().toSec() - t_init;

            //Save beacons data
            fprintf(beacons_log,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",z_beacons(0),z_beacons(1),z_beacons(2),z_beacons(3),z_beacons(4),z_beacons(5),z_beacons(6),time_log);
            fprintf(beaconsRaw_log,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",z_beacons_raw(0),z_beacons_raw(1),z_beacons_raw(2),z_beacons_raw(3),z_beacons_raw(4),z_beacons_raw(5),z_beacons_raw(6),time_log);
          }

        }
      }
    }




    if(enable_slam){
      // ========================================
      // Update filter with information from slam
      // ========================================
      if(new_z_slam == true){
        new_z_slam = false;
        count_slam++;

        if (count_slam == slam_step){
          count_slam = 0;

          if (filter_init==true){

            //Transform pose with quaternion to pose with Euler angle
            pos_euler.block(0,0,3,1) = z_slam.block(0,0,3,1);
            pos_euler.block(3,0,3,1) = Filter->quat2eulerangle(z_slam.block(3,0,4,1));

            //Call the update
            Filter->callback_pose_slam(pos_euler);

            //Save data to log files
            if(enable_txt_log){

              time_log = ros::Time::now().toSec() - t_init;

              //Save slam data
              fprintf(slam_log,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",z_slam(0),z_slam(1),z_slam(2),z_slam(3),z_slam(4),z_slam(5),z_slam(6),time_log);
            }

          }
        }
      }
    }






    if(filter_init){
      // ----------  ----------  ---------- ----------  ----------
      //Publish the pose estimation of the robot

      // Uptade the rost variable to be publishe
      drone_pose.position.x = states(0);
      drone_pose.position.y = states(1);
      drone_pose.position.z = states(2);
      drone_pose.orientation.w = states(3);
      drone_pose.orientation.x = states(4);
      drone_pose.orientation.y = states(5);
      drone_pose.orientation.z = states(6);


      drone_odom.header.frame_id = "world";
      drone_odom.header.stamp = ros::Time::now();
      drone_odom.pose.pose.position.x = states(0);
      drone_odom.pose.pose.position.y = states(1);
      drone_odom.pose.pose.position.z = states(2);
      drone_odom.pose.pose.orientation.w = states(3);
      drone_odom.pose.pose.orientation.x = states(4);
      drone_odom.pose.pose.orientation.y = states(5);
      drone_odom.pose.pose.orientation.z = states(6);

      Eigen::Quaterniond quat_states(states(3),states(4),states(5),states(6));
      R = quat_states.toRotationMatrix();
      R_ = R_bw(states.block(3,0,4,1));
      velo_w = R*states.block(7,0,3,1);



      drone_odom.twist.twist.linear.x = velo_w(0);
      drone_odom.twist.twist.linear.y = velo_w(1);
      drone_odom.twist.twist.linear.z = velo_w(2);



      // Publish the Pose message
      // pose_pub.publish(drone_pose);
      odom_pub.publish(drone_odom);


      // ----------  ----------  ---------- ----------  ----------
      //Publish a marker with the pose of the estimation
      drone_marker.header.frame_id = "/world";
      drone_marker.header.stamp = ros::Time::now();
      drone_marker.id = 0;
      drone_marker.type = drone_marker.CUBE;
      drone_marker.action = drone_marker.ADD;
      drone_marker.scale.x = 0.5;
      drone_marker.scale.y = 0.5;
      drone_marker.scale.z = 0.1;
      drone_marker.color.a = 0.5;
      drone_marker.color.r = 0.3;
      drone_marker.color.g = 0.3;
      drone_marker.color.b = 0.3;
      drone_marker.pose.position.x = states(0);
      drone_marker.pose.position.y = states(1);
      drone_marker.pose.position.z = states(2);
      drone_marker.pose.orientation.w = states(3);
      drone_marker.pose.orientation.x = states(4);
      drone_marker.pose.orientation.y = states(5);
      drone_marker.pose.orientation.z = states(6);

      marker_pub.publish(drone_marker);
      // ----------  ----------  ---------- ----------  ----------

      //Publish a transform between the world frame and the filter estimation
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(states(0), states(1), states(2)) );
      tf::Quaternion q(states(4),states(5),states(6),states(3));

      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/uav/estimation"));
    }





    //Check callbacks and wait
    ros::spinOnce();
    loop_rate.sleep();


    //Publish states in a vector
    std_msgs::Float32MultiArray vector_states;
    for (int k = 0; k<10; k++){
      vector_states.data.push_back(states(k));
    }
    states_pub.publish(vector_states);



  }//end while

  if(enable_txt_log){
    //Close log files
    fclose(state_log);
    fclose(imu_log);
    fclose(beacons_log);
    fclose(slam_log);
    fclose(beaconsRaw_log);
  }
}
