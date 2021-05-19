#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>
#include <mav_msgs/RateThrust.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ros/package.h>

#include <planning/Coefs.h>
#include <eigen3/Eigen/Dense>

#define USE_PY
//#define USE_TXT

//Parameters of the code
#define pi 3.14159265 //pi
#define g 9.81 //gravity
#define GOLD 0.6180339887 //golden ratio minus one
#define n 100 //number of points representing a curve
#define freq 120.0 //control loop frequency
#define Ts 1.0/freq
#define POLY_ORDER 10 //order of the polynomial

#define v_r 10.0 //robots reference speed


//Global parameters to be read from the config file
// Drone mass
double m = 3.3; //m = 3.2; 
// Vector field gain
double Kp = 1.0;
// Proportional velocity gains
double Kx = 0.7;
double Ky = 0.7;
double Kz = 0.7;
// Gain of the orientation controler
double Kr = 3.0;
// Integral vertical gain
double Kz_int = 0.0;
// Drag coeff
double DRAG_COEF = 0.05;
// Trajectory Selection -  0 = Polynomial, 1 = Auruco Circle, 2 = Circle, 3 = Ellipse, 4 = Square
int curve_selection = 0;
// Ratio velocity
double RATIO = 5.0;
// Drone Speed: If <= 0 - polynomial planned velocity, If > 0 fixed to informed speed - ex: 1.0, 1.5, 2.0 ....
double drone_speed = -1.0;
// Yaw type: 1 - Fixed yaw (front),  2 - Drone x axis aligned with the field,  Any other value - Left Pair cameras aligned with the field
int yaw_type = 1;




//miguezao temporario
double aaa[100][2];




using namespace std;
using namespace Eigen;

//Class for computing the integrals of the error
class IntErro{
  public:
  IntErro(): vx(0),vy(0),vz(0),phi(0),theta(0),psi(0){}

  //Integral error for velocity
  double vx;
  double vy;
  double vz;

  //Integral error for orientation
  double phi; // roll
  double theta; // pitch
  double psi; // yaw
};
//Object for the integral of the error
IntErro I_erro;



//Class for the drone states
class States{
  public:
    //Constructor
    States(): x(0),y(0),z(0),vx(0),vy(0),vz(0),phi(0),theta(0),psi(0),qw(0),qx(0),qy(0),qz(0),ax(0),ay(0),az(0){}

    //Position
    double x;
    double y;
    double z;

    //Velocity in the world frame
    double vx;
    double vy;
    double vz;

    //Euler angles
    double phi; // roll
    double theta; // pitch
    double psi; // yaw

    //Quaternion
    double qw;
    double qx;
    double qy;
    double qz;

    //Reference acceleration
    double ax;
    double ay;
    double az;

};
//Object for the drone states
States state_now;





//Function to read the parameters of the curve
void read_parameters(ros::NodeHandle nh){

  try{
    //Read the parameters
    nh.getParam("m", m);
    nh.getParam("Kp", Kp);
    nh.getParam("Kx", Kx);
    nh.getParam("Ky", Ky);
    nh.getParam("Kz", Kz);
    nh.getParam("Kr", Kr);
    nh.getParam("Kz_int", Kz_int);
    nh.getParam("DRAG_COEF", DRAG_COEF);
    nh.getParam("curve_selection", curve_selection);
    nh.getParam("RATIO", RATIO);
    nh.getParam("drone_speed", drone_speed);
    nh.getParam("yaw_type", yaw_type);

    //Print the parameters on the screen
    if(true){
      printf("\33[92m The following parameters were loaded:\33[0m\n");
      printf("\33[94m m: %f\33[0m\n",m);
      printf("\33[94m Kp: %f\33[0m\n",Kp);
      printf("\33[94m Kx: %f\33[0m\n",Kx);
      printf("\33[94m Ky: %f\33[0m\n",Ky);
      printf("\33[94m Kz: %f\33[0m\n",Kz);
      printf("\33[94m Kr: %f\33[0m\n",Kr);
      printf("\33[94m Kz_int: %f\33[0m\n",Kz_int);
      printf("\33[94m DRAG_COEF: %f\33[0m\n",DRAG_COEF);
      printf("\33[94m curve_selection: %d\33[0m\n",curve_selection);
      printf("\33[94m RATIO: %f\33[0m\n",RATIO);
      printf("\33[94m drone_speed: %f\33[0m\n",drone_speed);
      printf("\33[94m yaw_type: %d\33[0m\n",yaw_type);
    }
  } catch(...){
    //Inform error when reading the parameters
    printf("\33[41mProblem occurred when trying to read the parameters - field_controller.cpp!\33[0m\n");
  }

}

bool flag_position = false;


// This is the callback for Pose
void GetEkf(const nav_msgs::Odometry::ConstPtr &msg) {

  double r, p, y;
  tf::Matrix3x3 rotMatrix(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));

  // Get roll, pitch and yaw
  rotMatrix.getRPY(r, p, y);
  state_now.phi = r;
  state_now.theta = p;
  state_now.psi = y;

  // Compute velocity
  state_now.vx = msg->twist.twist.linear.x;
  state_now.vy = msg->twist.twist.linear.y;
  state_now.vz = msg->twist.twist.linear.z;

  // Get position
  state_now.x = msg->pose.pose.position.x;
  state_now.y = msg->pose.pose.position.y;
  state_now.z = msg->pose.pose.position.z;

  // Get quaternion
  state_now.qw = msg->pose.pose.orientation.w;
  state_now.qx = msg->pose.pose.orientation.x;
  state_now.qy = msg->pose.pose.orientation.y;
  state_now.qz = msg->pose.pose.orientation.z;

  flag_position = true;
}



//Class to represent a polynomial trajectory
class Curve{
  public:
  double Tf;
  double x[POLY_ORDER+1];
  double y[POLY_ORDER+1];
  double z[POLY_ORDER+1];
  double fh;
};
//Vector of polynomial curves
vector<Curve> m_curves(POLY_ORDER+1);


bool flag_start = false;
bool flag_coefs = false;


//This function is only used to read polynomial parameters from a txt file
//It is not used if the coefficients come via ros topic
bool readCurvesFile(std::string fileName, vector<Curve> &curves )
{
  bool debug = true;
  curves.clear();

  ifstream curvesFile(fileName.c_str());

  if (!curvesFile.is_open()){
    cout << "\33[41mUnable to open file:\33[41m "<< fileName.c_str() << endl;
    return false;
  }

  try{
    // Read curves size
    int curvesSize;
    curvesFile >> curvesSize;

    if(debug)
      cout << "curvesSize = " << curvesSize << endl;

    double fh_vec[] = {0.0539617337863,
                      0.0877098917208,
                      0.122356413599,
                      0.081301364153,
                      0.060277771055,
                      0.177417333065,
                      0.149031712067,
                      0.389933303229,
                      0.137899526869,
                      0.050749954898,
                      0.0630287558055};
    for( int i = 0; i < curvesSize; i++ )
    {
      Curve curve;
      curvesFile >> curve.Tf;
      curve.fh = fh_vec[i];

      if(debug)
  cout << "curve " << i+1 << ": Tf = " << curve.Tf << endl;

      for( int j = 0; j < POLY_ORDER+1; j++ ){
        curvesFile >> curve.x[j];}
      for( int j = 0; j < POLY_ORDER+1; j++ ){
        curvesFile >> curve.y[j];}
      for( int j = 0; j < POLY_ORDER+1; j++ ){
        curvesFile >> curve.z[j];}

      curves.push_back(curve);
    }
  }
  catch (exception& e){
    cout << "Standard exception: " << e.what() << endl;
  }

  return true;
}




// Callback Polynomial Coefs
void callback_coefs(const planning::Coefs::ConstPtr &msg){
  m_curves[msg->curve_id].Tf = msg->T;
  m_curves[msg->curve_id].fh = msg->fh;
  for (int k=0; k<=POLY_ORDER; k++){
    m_curves[msg->curve_id].x[k] = msg->cx[k];
    m_curves[msg->curve_id].y[k] = msg->cy[k];
    m_curves[msg->curve_id].z[k] = msg->cz[k];
  }
  flag_coefs = true;
}



// Callback to initialize the controller (Beacons are alread available)
void GetStart(const std_msgs::Bool::ConstPtr &msg) {
    if(msg->data){
       flag_start = true;
    }
}


// Function to evaluate the polynomial
void sample_curve(double s, int curve_now, double *C, double *v_poly){

  double V[3];

  C[0] = 0.0;
  C[1] = 0.0;
  C[2] = 0.0;

  V[0] = 0.0;
  V[1] = 0.0;
  V[2] = 0.0;

///*
  for(int k=0; k<POLY_ORDER+1; k++){
    //Compute polynomial position
    C[0] += m_curves[curve_now].x[k]*pow(s,k);
    C[1] += m_curves[curve_now].y[k]*pow(s,k);
    C[2] += m_curves[curve_now].z[k]*pow(s,k);
    //Compute polynomial velocity
    if(k>0){
      V[0] += k*m_curves[curve_now].x[k]*pow(s,k-1);
      V[1] += k*m_curves[curve_now].y[k]*pow(s,k-1);
      V[2] += k*m_curves[curve_now].z[k]*pow(s,k-1);
    }
  }

  // *v_poly = sqrt(V[0]*V[0] + V[1]*V[1] + V[2]*V[2]) / RATIO; // (1.8+rnd*0.4*0);


  *v_poly = drone_speed; 

  
}



// Golden search optimization method
void aurea(double al, double bl, int curve_now, double *x, double *s_star){

  double dist_vec[3];
  double ra[3], rb[3];
  double func_a, func_b;
  double s_a, s_b;
  double v_poly;

  s_a = bl - GOLD*(bl-al);
  s_b = al + GOLD*(bl-al);

  sample_curve(s_a, curve_now, ra, &v_poly);
  dist_vec[0] = x[0]-ra[0]; dist_vec[1] = x[1]-ra[1]; dist_vec[2] = x[2]-ra[2];
  func_a = sqrt(dist_vec[0]*dist_vec[0] + dist_vec[1]*dist_vec[1] + dist_vec[2]*dist_vec[2]);

  sample_curve(s_b, curve_now, rb, &v_poly);
  dist_vec[0] = x[0]-rb[0]; dist_vec[1] = x[1]-rb[1]; dist_vec[2] = x[2]-rb[2];
  func_b = sqrt(dist_vec[0]*dist_vec[0] + dist_vec[1]*dist_vec[1] + dist_vec[2]*dist_vec[2]);

  int k=0;
  while(bl-al > 0.000000001){

    k = k+1;
    if (k==1000) {break;}

    if(func_a > func_b){
      al = s_a;
      s_a = s_b;
      s_b = al + GOLD*(bl-al);
    }
    else{
      bl = s_b;
      s_b = s_a;
      s_a = bl - GOLD*(bl-al);
    }

    sample_curve(s_a, curve_now, ra, &v_poly);
    dist_vec[0] = x[0]-ra[0]; dist_vec[1] = x[1]-ra[1]; dist_vec[2] = x[2]-ra[2];
    func_a = sqrt(dist_vec[0]*dist_vec[0] + dist_vec[1]*dist_vec[1] + dist_vec[2]*dist_vec[2]);

    sample_curve(s_b, curve_now, rb, &v_poly);
    dist_vec[0] = x[0]-rb[0]; dist_vec[1] = x[1]-rb[1]; dist_vec[2] = x[2]-rb[2];
    func_b = sqrt(dist_vec[0]*dist_vec[0] + dist_vec[1]*dist_vec[1] + dist_vec[2]*dist_vec[2]);
  }
  *s_star = (al+bl)/2;
}




// Function to compute the vector field
void vector_field(double x, double y, double z, int curve_now, double *vx, double *vy, double *vz, double *V, bool &flag_curve_now, bool &flag_orientation_now){

  double s[n];
  double C[3][n], C_pl[3][n], C_[3], C_aux[3];
  double ds = 0;
  double Tf = 0.0;
  double R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  double pos[3] = {x, y, z};
  double r_star[3], r_star_pl[3], r_star_M[3], r_star_m[3];
  double delta_s = 0.000001;

  double s_star, s_star_pl;
  double v_poly, lixo;

  // Get the final time of the polynomial
  Tf = m_curves[curve_now].Tf;
  ds = Tf/n;


  if (curve_selection > 0){
    Tf = 2*M_PI;
  }
  else{
    Tf = m_curves[curve_now].Tf;
  }
  ds = 1.05*Tf/(n);

  // Create a vector of parameters (of instants)
  s[0] = - m_curves[curve_now].Tf/20.0;
  for (int k=1; k<n; k++){
    s[k] = s[k-1]+ds;
  }



  //Evaluate the curve thrugh the parameters
  for (int k=0; k<n; k++){
    sample_curve(s[k], curve_now, C_aux, &lixo);
    C[0][k]=C_aux[0]; C[1][k]=C_aux[1]; C[2][k]=C_aux[2];
  }

  // Compute the closest point in the curve and the distance to it
  int s_cl = 0;
  double D = sqrt(pow(C[0][s_cl]-x,2) + pow(C[1][s_cl]-y,2) + pow(C[2][s_cl]-z,2));
  for (int i = 1; i<n; i++){
    if (sqrt(pow(C[0][i]-x,2) + pow(C[1][i]-y,2) + pow(C[2][i]-z,2)) < D){
      D = sqrt(pow(C[0][i]-x,2) + pow(C[1][i]-y,2) + pow(C[2][i]-z,2));
      s_cl = i; //index of the closest point
    }
  }
  // Use the golden search method to refine the solution
  aurea(s[s_cl]-1.1*ds, s[s_cl]+1.1*ds, curve_now,  pos, &s_star);
  // Compute the curve at the closest point
  sample_curve(s_star, curve_now, r_star, &v_poly);
  // Compute the distance vector and the distance function
  double Dv[3] = {x-r_star[0], y-r_star[1], z-r_star[2]};
  D = sqrt(Dv[0]*Dv[0] + Dv[1]*Dv[1] + Dv[2]*Dv[2]);
  // Compute the gradient of the distance function
  double grad_D[3] = {Dv[0]/(D+0.000001), Dv[1]/(D+0.000001), Dv[2]/(D+0.000001)};
  //cout << "s_cl   = " << s_cl << endl;
  //cout << "s_star = " << s_star << endl;

  // Evaluate the curve at neighborhood
  sample_curve(s_star+delta_s, curve_now, r_star_M, &lixo);
  sample_curve(s_star-delta_s, curve_now, r_star_m, &lixo);

  //Compute the Tangent vector
  double norma=0;
  double T[3] = {(r_star_M[0]-r_star_m[0])/(2*delta_s), (r_star_M[1]-r_star_m[1])/(2*delta_s), (r_star_M[2]-r_star_m[2])/(2*delta_s)};
  norma = sqrt(T[0]*T[0] + T[1]*T[1] + T[2]*T[2]);
  T[0] = T[0]/(norma+0.0000001);
  T[1] = T[1]/(norma+0.0000001);
  T[2] = T[2]/(norma+0.0000001);

  // Compute the gain functions of the vector field
  double G = -(2/pi)*atan(Kp*D);
  double H = sqrt(1-G*G);

  // Compute the convergent vector
  double Psi_g[3] = {G*grad_D[0], G*grad_D[1], G*grad_D[2]};
  // Compute the tangent vector
  double Psi_h[3] = {H*T[0], H*T[1], H*T[2]};

  // Compute the scalling factor
  double eta = v_poly;

  // Compute the scalled field
  double Phi[3] = {eta*(Psi_g[0]+Psi_h[0]), eta*(Psi_g[1]+Psi_h[1]), eta*(Psi_g[2]+Psi_h[2]) };

  //Get outputs
  *V = D;
  // *Eta = eta;

  //Give the velocities
  *vx = Phi[0];
  *vy = Phi[1];
  *vz = Phi[2];

  //Indicate end of curve and command the change of polynomial
  if (s_cl >= n-1 && flag_curve_now == false){
    flag_curve_now = true;
  }
  else if ( s_cl >= n-1 - round(m_curves[curve_now].fh * n)  && flag_orientation_now == false){
      flag_orientation_now = true;
  }

}


//Second order field, gives a reference acceleration
void get_acc(Vector3d pos0, Vector3d vel0, int curve_now, int orientation_now, Vector3d& a_r, double *psi_r, double tau, Vector3d z_b, double delta){


  double x_rob, y_rob, z_rob, x_dot, y_dot, z_dot;

  x_rob = pos0(0); y_rob = pos0(1); z_rob = pos0(2);
  x_dot = vel0(0); y_dot = vel0(1); z_dot = vel0(2);

  Vector3d z_hat; z_hat << 0,0,1;

  // Compute the drag compensation
  // Drag type 1
  double norm_vel = sqrt(x_dot*x_dot + y_dot*y_dot + z_dot*z_dot);
  double F_drag_x = -1*DRAG_COEF*x_dot*norm_vel;
  double F_drag_y = -1*DRAG_COEF*y_dot*norm_vel;
  double F_drag_z = -1*DRAG_COEF*z_dot*norm_vel;

  Vector3d f_drag;
  f_drag << F_drag_x, F_drag_y, F_drag_z;

  // Compute a delta to further compute the derivative
  Vector3d pos, vel;
  pos = pos0 + vel0*delta;
  vel = vel0 + (z_b*tau/m -g*z_hat + f_drag/m)*delta;

  x_rob = pos(0); y_rob = pos(1); z_rob = pos(2);
  x_dot = vel(0); y_dot = vel(1); z_dot = vel(2);

  double vx, vy, vz;
  double vxMX, vyMX, vzMX, vxMY, vyMY, vzMY, vxMZ, vyMZ, vzMZ;
  double ax, ay, az;
  double Jac[3][3];
  double V;
  static double int_erro_vz = 0.0;

  bool lixo1 = false;
  bool lixo2 = false;
  ///*
  // Evaluate the vector field at current position
  vector_field(x_rob, y_rob, z_rob, curve_now, &vx, &vy, &vz, &V, lixo1, lixo2);
  // Evaluate the vector field at the neighborhood of the current position
  vector_field(x_rob + 0.0001, y_rob, z_rob, curve_now, &vxMX, &vyMX, &vzMX, &V, lixo1, lixo2);
  vector_field(x_rob, y_rob + 0.0001, z_rob, curve_now, &vxMY, &vyMY, &vzMY, &V, lixo1, lixo2);
  vector_field(x_rob, y_rob, z_rob + 0.0001, curve_now, &vxMZ, &vyMZ, &vzMZ, &V, lixo1, lixo2);

  // Compute the reference for yaw
  if(yaw_type == 0){
    *psi_r = 0; // fixed orintation
  }
  else if (yaw_type == 1){
    *psi_r = 0 - (pi/2); // fixed yaw
    //*psi_r = 3.1415926535*(5.0/6.0); // fixed yaw
    *psi_r = atan2(vy,vx);
  }
  else if(yaw_type == 2){
    double x_gate = aaa[orientation_now][0];
    double y_gate = aaa[orientation_now][1];
    *psi_r = atan2(y_gate-state_now.y, x_gate-state_now.x);
    

  }
  else{
    *psi_r = 0; // left cameras aligned with the field
  }


  // Compute the field's Jacobian numerically
  Jac[0][0] = (vxMX-vx)/0.0001;   Jac[0][1] = (vxMY-vx)/0.0001;   Jac[0][2] = (vxMZ-vx)/0.0001;
  Jac[1][0] = (vyMX-vy)/0.0001;   Jac[1][1] = (vyMY-vy)/0.0001;   Jac[1][2] = (vyMZ-vy)/0.0001;
  Jac[2][0] = (vzMX-vz)/0.0001;   Jac[2][1] = (vzMY-vz)/0.0001;   Jac[2][2] = (vzMZ-vz)/0.0001;

  // Compute the linear accelerations for the feed-forward term of the controller
  ax = Jac[0][0]*x_dot + Jac[0][1]*y_dot + Jac[0][2]*z_dot + Kx*(vx-x_dot) - F_drag_x/m;
  ay = Jac[1][0]*x_dot + Jac[1][1]*y_dot + Jac[1][2]*z_dot + Ky*(vy-y_dot) - F_drag_y/m;
  az = Jac[2][0]*x_dot + Jac[2][1]*y_dot + Jac[2][2]*z_dot + Kz*(vz-z_dot) - F_drag_z/m + Kz_int*int_erro_vz + g;

  int_erro_vz = int_erro_vz + (vz-z_dot)*(1.0/freq);

  if(fabs(Kz_int*int_erro_vz) > 0.7){
    int_erro_vz = 0.7/Kz_int * ((int_erro_vz)/(fabs(int_erro_vz)));
  }

    a_r << ax, ay, az;


}//get_acc()



// Compute a roientation matrix
Matrix3d get_orientation_ref(Vector3d a_r, double psi_r){

  Vector3d x_r, y_r, z_r, w_psi;
  Matrix3d Rr;
  z_r = a_r/(a_r.norm() + 0.00000000001);
  w_psi << cos(psi_r), sin(psi_r), 0;
  x_r = (w_psi - w_psi.dot(z_r)*z_r)/((w_psi - w_psi.dot(z_r)*z_r).norm() + 0.00000000001);
  y_r = z_r.cross(x_r);
  Rr << x_r, y_r, z_r;

  return Rr;
}







///*
void XQuad_controller(Vector3d pos, Vector3d vel, Matrix3d R, int *curve_now, int *orientation_now, double *tau, Vector3d& omega, double *Dist, Vector3d& vel_ref, bool *flag_terminate, Matrix3d& Rr){


  //Get the z axis in the body and in the world frame
  Vector3d z_b, z_hat;
  z_b = R.block(0,2,3,1);
  z_hat << 0, 0, 1;


  double fx, fy, fz, V;
  // Flag to indicate end of polynomial
  bool flag_curve_now = false;
  bool flag_orientation_now = false;
  // Computation of the vector field
  vector_field(pos(0), pos(1), pos(2), *curve_now, &fx, &fy, &fz, &V, flag_curve_now, flag_orientation_now);
  *Dist = V;

  vel_ref << fx, fy, fz;

  double psi_r, psi_r_M, psi_r_m;

  // Computation of reference orientation
  Vector3d a_r;
//  Matrix3d Rr;
//  get_acc(pos, vel, *curve_now, a_r, &psi_r, 0.0, z_b, 0.0);
  get_acc(pos, vel, *curve_now, *orientation_now, a_r, &psi_r, 0.0, z_b, 0.0);
  Rr =  get_orientation_ref(a_r, psi_r);
  //cout << "Rr: \n" << Rr << endl;

  // // Control signal for thrust
  *tau = m*a_r.dot(z_b);

  // Computation of the derivative of the matrix Rr
  double delta_t = (1.0)/freq;
  Vector3d a_r_M;
  Matrix3d Rr_M;
  get_acc(pos, vel, *curve_now, *orientation_now, a_r_M, &psi_r_M, *tau, z_b, delta_t);
  Rr_M =  get_orientation_ref(a_r_M, psi_r_M);
  Vector3d a_r_m;
  Matrix3d Rr_m;
  get_acc(pos, vel, *curve_now, *orientation_now, a_r_m, &psi_r_m, *tau, z_b, -delta_t);
  Rr_m =  get_orientation_ref(a_r_m, psi_r_m);

  Matrix3d Re, Rr_dot, S_w;

  // Orientation error
  Re = R.transpose()*Rr;

  // Time derivative of Rr
  Rr_dot = (Rr_M-Rr_m)/(2*delta_t);

  S_w = R.transpose()*Rr_dot;
  S_w = S_w*(Re.transpose());

  // Computation of the feedforward angular velocity
  Vector3d omega_d;
  omega_d << S_w(2,1)-S_w(1,2), S_w(0,2)-S_w(2,0), S_w(1,0)-S_w(0,1);
  omega_d = omega_d/2.0;

  // Computation of the axis angle representation
  Vector3d axis;

  AngleAxisd angle_axis(Re);
  double alpha = angle_axis.angle();
  axis = angle_axis.axis();

  omega = omega_d + Kr*alpha*axis;


  if(flag_curve_now){
   *curve_now = *curve_now+1;
   if (*curve_now >= (int)m_curves.size() ){
    *flag_terminate = true;
   }
 }
 else if(flag_orientation_now && *curve_now==*orientation_now){
    *orientation_now = *orientation_now+1;
 }

} // end XQuad_controller





// Main
int main(int argc, char **argv) {

  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  ros::NodeHandle nh2("controller");
  read_parameters(nh2);

  
  ros::Subscriber ekf_sub = nh.subscribe<nav_msgs::Odometry>("/ekf/odom", 1, GetEkf);
  ros::Subscriber start_sub = nh.subscribe<std_msgs::Bool>("/flag_start", 1, GetStart);
#ifdef USE_PY //Use trajectory from python code
  ros::Subscriber coefs_sub = nh.subscribe<planning::Coefs>("/planner/coefs", 1, callback_coefs);
#endif

  ros::Publisher arm_pub = nh.advertise<std_msgs::Empty>("/uav/input/arm", 1);
  ros::Publisher thrust_rate_pub = nh.advertise<mav_msgs::RateThrust>("/uav/input/rateThrust", 1);
  ros::Publisher curve_now_pub = nh.advertise<std_msgs::Int32>("/path/curve_now", 1);
  ros::Rate loop_rate(freq);

  // Parameters read
  string resultsFile;
  nh.getParam ("/field_follow/resultsFile", resultsFile);


  srand( (unsigned)time( NULL ) );

#ifdef USE_TXT //Use trajectory from txt file
  string curvesFile;
  nh.param<std::string>( "/field_follow/curvesFile", curvesFile, "./src/window_1/text/coefs_info.txt" );

  if( !readCurvesFile(curvesFile,m_curves) ){
    cout << "Can't read file: " << curvesFile << endl;
    return -1;
  }
  flag_coefs = true;
#endif

  geometry_msgs::Twist quad_velocity;
  mav_msgs::RateThrust quad_rateThrust;
  std_msgs::Int32 curve_now_int;
  double vx, vy, vz, omega_z, V, Eta;
  double vxMX, vyMX, vzMX, vxMY, vyMY, vzMY, vxMZ, vyMZ, vzMZ;
  double J[3][3];
  double time = 0;
  double norm_Psi_t=0;

  //Controller
  double u1, phi_d, theta_d, psi_d;
  double acc_r_x, acc_r_y, acc_r_z;
  double phi_r, theta_r, psi_r;
  phi_r = 0;
  theta_r = 0;
  psi_r = 0;

  double psi_r_0, psi_r_MX, psi_r_MY, psi_r_MZ, grad_psi_r[3];
  double psi_r_dot_ff;

  double qx,qy,qz,qw; //quaternion
  double R[3][3]; //rotation matrix
  double v_rot[3]; //velocity transformed to the body axes

  bool flag_terminate = false;

  Vector3d vel_ref;
  vel_ref << 0.0, 0.0, 0.0;
  Matrix3d R_ref;
  R_ref << 1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0;

  Vector3d omega;
  double tau;
  omega << 0.0, 0.0, 0.0;
  tau = 0.0;

  double D;

  vx = 0;
  vy = 0;
  vz = 0.0;

  omega_z = 0.1;

  // Initialize the robot following the first polynomial
  int curve_now = 0;
  int orientation_now = 0;


  double t_init = ros::Time::now().toSec();
  double time_log = 0.0;
  FILE *result_file;
  result_file = fopen(resultsFile.c_str(),"w");



  //GET GATE PARAMETERS
  std::string key;
  XmlRpc::XmlRpcValue gates;
  nh.getParam("/uav/gate_names", gates);
  //double aaa[gates.size()][2];
  for (int i = 0; i < gates.size(); i++){

      std::string param = "/uav/" + std::string(gates[i]) + "/nominal_location";
      if (nh.searchParam(param, key))
      {
        XmlRpc::XmlRpcValue parameters;
        nh.getParam(key, parameters);
        double x_ = 0;
        double y_ = 0;
        for (int j = 0; j < 4; j++){
            //std::cout << double(parameters[j][0]);
            x_ += double(parameters[j][0]);
            y_ += double(parameters[j][1]);

        }
        aaa[i][0] = x_/4;
        aaa[i][1] = y_/4;
      }
   }
  // GETTING THE OTHER PARAMETERS

   // MIGUEZAO - Create a new imaginary gate in the end
   aaa[gates.size()][0] = 2*aaa[gates.size()-1][0] - aaa[gates.size()-2][0];
   aaa[gates.size()][1] = 2*aaa[gates.size()-1][1] - aaa[gates.size()-2][1];

  XmlRpc::XmlRpcValue init_pose;
  nh.getParam("/uav/flightgoggles_uav_dynamics/init_pose", init_pose);


  tf::Quaternion q_( (double(init_pose[3])) , (double(init_pose[4])) ,  (double(init_pose[5])),  (double(init_pose[6])));
  tf::Matrix3x3 m_aux(q_);
  double roll_, pitch_, yaw_;
  m_aux.getRPY(roll_, pitch_, yaw_);

  double START_X = double(init_pose[0]);
  double START_Y = double(init_pose[1]);
  double START_Z = double(init_pose[2]);
  double START_YAW = yaw_;


  XmlRpc::XmlRpcValue drag_coefficient;
  nh.getParam("/uav/flightgoggles_uav_dynamics/drag_coefficient", drag_coefficient);
  if(DRAG_COEF != -1){
    DRAG_COEF = double(drag_coefficient);
  }
  

  XmlRpc::XmlRpcValue mass;
  nh.getParam("/uav/flightgoggles_uav_dynamics/vehicle_mass", mass);
  m = double(mass);



  int state = 0; // 0 - take off; 1 - follow vector field

  while (ros::ok()){

    // Read the callbacks
    ros::spinOnce();

    //Create some variables
    Vector3d pos, vel;
    Quaterniond quat;
    Matrix3d R;
    pos << state_now.x, state_now.y, state_now.z;
    vel << state_now.vx, state_now.vy, state_now.vz;
    quat.w() = state_now.qw;
    quat.x() = state_now.qx;
    quat.y() = state_now.qy;
    quat.z() = state_now.qz;
    R = quat.toRotationMatrix();

    //State to take off in the simulation
    if (state == 0){
      vx = 0;
      vy = 0;
      vz = 0;
      acc_r_x = 0;
      acc_r_y = 0;
      acc_r_z = 0;
      state = 0;

      omega << 0.0, 0.0, 0.0;
      tau = 0.0;
      tau = 1.2*m*g;

      //cout << "flag_start = " << flag_start << ", flag_coefs = " << flag_coefs << endl;
      if(flag_start==true && flag_coefs==true && flag_position==true){
         state = 2;
         std_msgs::Empty arm_msg;
      }
    }

    //State to go to an initial pose
    else if (state == 1){

      // Compute commands to go to initial position
      psi_r = START_YAW;
      vx = 1.5*(START_X-state_now.x);
      vy = 1.5*(START_Y-state_now.y);
      vz = 1.5*(START_Z-state_now.z);
      acc_r_x = -1.5*state_now.vx;
      acc_r_y = -1.5*state_now.vy;
      acc_r_z = -1.5*state_now.vz;

      double dist = sqrt(pow(START_X-state_now.x,2) + pow(START_Y-state_now.y,2) + pow(START_Z-state_now.z,2));

      if (dist <= 1.0){  // initial position achieved
        state=2;
        quad_velocity.linear.x = 0.0;
        quad_velocity.linear.y = 0.0;
        quad_velocity.linear.z = 0.0;
      }
      printf("\nGoing to initial position!\nDistance: %f\nTime: %f\n",dist, time);

      curve_now_int.data = curve_now;
      curve_now_pub.publish(curve_now_int);
    }


    // State to follow the vector field
    else if (state == 2){

      time = time + 1.0/freq; //count time

      // Flag to indicate end of polynomial
      bool flag_curve_now = false;
      bool flag_orientation_now = false;



      XQuad_controller(pos, vel, R, &curve_now, &orientation_now, &tau, omega, &D, vel_ref, &flag_terminate, R_ref);


      // Print distance to the curve
      cout << "\33[40mD: " << D << "\33[0m" << endl;
      cout << "\33[99mcurve_now: " << curve_now << "\33[0m" << endl;
      cout << "\33[40morien_now: " << orientation_now << "\33[0m" << endl;
      cout << "\33[99mpos: " << pos.transpose() << "\33[0m" << endl;
      cout << "\33[40mvel_r: " << vel_ref.transpose() << "\t" << "|vel_r|: " << vel_ref.norm() <<"\33[0m" << endl;
      cout << "\33[99mvel:   " << vel.transpose() << "\t" << "|vel|: " << vel.norm() << "\33[0m" << endl;
      cout << "\33[40mtau: " << tau << "\33[0m" << endl;
      cout << "\33[99momega:   " << omega.transpose() << "\33[0m" << endl;
      cout << endl;



      Quaterniond quat_ref(R_ref);

      time_log = ros::Time::now().toSec() - t_init;
      fprintf(result_file,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",pos(0),pos(1),pos(2),vel_ref(0),vel_ref(1),vel_ref(2),vel(0),vel(1),vel(2),quat_ref.w(),quat_ref.x(),quat_ref.y(),quat_ref.z(),state_now.qw,state_now.qx,state_now.qy,state_now.qz,D,omega(0),omega(1),omega(2),tau,time_log);
      fflush(result_file);
      


      curve_now_int.data = orientation_now;
      curve_now_pub.publish(curve_now_int);
      
      // If end of task go to the stop state
      if(flag_terminate){
        state = 3;
      }

    }

    // State to stop the drone
    else if (state == 3){

      // Command the drone to stay sttoped
      vx = 0.0;
      vy = 0.0;
      vz = 0.0;
      acc_r_x = 0.0;
      acc_r_y = 0.0;
      acc_r_z = 0.0;
      psi_r = 0.0;
      psi_r_dot_ff = 0.0;

      omega << 0.0,0.0,0.0;
      tau = 0.0;

    }
    //End of states machine

    quad_rateThrust.angular_rates.x = omega(0);
    quad_rateThrust.angular_rates.y = omega(1);
    quad_rateThrust.angular_rates.z = omega(2);
    quad_rateThrust.thrust.z = tau;

    // Publish rateThrust command
    thrust_rate_pub.publish(quad_rateThrust);

    // Sleep program
    loop_rate.sleep();
  }


  //Terminate if ros is not ok
  if(!ros::ok()){
    // Close results file
    fclose(result_file);
    //fclose(f);
  }


}
