// #include <fftw3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <unsupported/Eigen/MatrixFunctions>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "usv_estiplan/Fftarray.h"
#include "usv_estiplan/Fftresult.h"
#include "usv_estiplan/Float64Stamped.h"
#include "usv_estiplan/PredictionOutput.h"
#include "usv_estiplan/Wavefuture.h"
using namespace std;

#define REAL 0
#define IMAG 1

const int STATE_COMPONENTS = 6;
const int OUTPUT_COMPONENTS = 3;
double sampling_freq = 100;
double sample_time = 1 / sampling_freq;
uint64_t iter_global = 0;
usv_estiplan::Fftarray fft_array{};
usv_estiplan::Fftarray ident_msg{};
double msg_time;
nav_msgs::Odometry usv_msg;
usv_estiplan::Float64Stamped decoded_msg;
usv_estiplan::Wavefuture wave_future_msg;
geometry_msgs::Pose temp_msg;
ros::Publisher wave_observer;
ros::Publisher wave_decoded;
ros::Publisher wave_future;
ros::Publisher pose_pred_pub;
string dof_name;
double t_future = 0.0;
double pred_horizon = 0.0;
bool horizon_loaded = false;
bool _diagnostics_ = false;
bool _online_tune_ = true;
Eigen::Matrix<double, OUTPUT_COMPONENTS, 1> w_k_hat =
    Eigen::MatrixXd::Zero(OUTPUT_COMPONENTS, 1);
Eigen::Matrix<double, OUTPUT_COMPONENTS, 1> w_k =
    Eigen::MatrixXd::Zero(OUTPUT_COMPONENTS, 1);
Eigen::Vector2d x_i;  // X(i,0) single mode
Eigen::Matrix<double, STATE_COMPONENTS, 1> x_t =
    Eigen::MatrixXd::Zero(STATE_COMPONENTS,
                          1);                              // X(t_0) mode vector
Eigen::MatrixXd l_k(STATE_COMPONENTS, OUTPUT_COMPONENTS);  // L_k Kalman gain
Eigen::Matrix<double, STATE_COMPONENTS, 1> x_predicted;    // X_k+1 predicted
Eigen::Matrix<double, STATE_COMPONENTS, 1> x_predicted_pred;  // X_k+1 predicted
// A(t_0)
Eigen::Matrix<double, STATE_COMPONENTS, STATE_COMPONENTS> a_t =
    Eigen::MatrixXd::Zero(STATE_COMPONENTS, STATE_COMPONENTS);
// Phi_0
Eigen::Matrix<double, STATE_COMPONENTS, STATE_COMPONENTS> phi =
    Eigen::MatrixXd::Identity(STATE_COMPONENTS, STATE_COMPONENTS);
// P_k+1 predicted
Eigen::Matrix<double, STATE_COMPONENTS, STATE_COMPONENTS> p_k_dash =
    Eigen::MatrixXd::Identity(STATE_COMPONENTS, STATE_COMPONENTS);
Eigen::Matrix<double, STATE_COMPONENTS, STATE_COMPONENTS> p_k_dash_pred =
    Eigen::MatrixXd::Identity(STATE_COMPONENTS, STATE_COMPONENTS);
// P_k+1 actual
Eigen::Matrix<double, STATE_COMPONENTS, STATE_COMPONENTS> p_k =
    Eigen::MatrixXd::Zero(STATE_COMPONENTS, STATE_COMPONENTS);

Eigen::Matrix<double, OUTPUT_COMPONENTS, STATE_COMPONENTS> c_t =
    Eigen::MatrixXd::Zero(3, STATE_COMPONENTS);
Eigen::Matrix2d a_i;  // A_i(t_0)
Eigen::Matrix<double, OUTPUT_COMPONENTS, OUTPUT_COMPONENTS> r =
    Eigen::MatrixXd::Zero(OUTPUT_COMPONENTS, OUTPUT_COMPONENTS);
Eigen::Matrix<double, STATE_COMPONENTS, STATE_COMPONENTS> q_dash;
Eigen::Matrix<double, STATE_COMPONENTS, STATE_COMPONENTS> q_dash_predict;
Eigen::Matrix<double, STATE_COMPONENTS, STATE_COMPONENTS> q =
    Eigen::MatrixXd::Zero(STATE_COMPONENTS, STATE_COMPONENTS);
bool first_start = true;
// row-0 is last, row-1 is current
// true is present, false is notpresent

Eigen::VectorXd phase(1);
Eigen::VectorXd freq(1);
Eigen::VectorXd amplitude(1);
// std::ofstream csv_debug;
Eigen::IOFormat CSVFormat(4, Eigen::DontAlignCols, ";", "\n");
ros::Publisher detected_output;

void OdomCallback(const geometry_msgs::PoseStamped &msg) {
  w_k(0) = msg.pose.position.x;
  w_k(1) = msg.pose.position.y;
  w_k(2) = msg.pose.position.z;
  if (_online_tune_) {
    ros::NodeHandle estiplan("~");
    if (!estiplan.getParam("r", r(0, 0))) {
      ROS_WARN("WAVE_PRED:KF_r loading failed, quitting");
    }
    if (!estiplan.getParam("r", r(1, 1))) {
      ROS_WARN("WAVE_PRED:KF_r loading failed, quitting");
    }
    if (!estiplan.getParam("r", r(2, 2))) {
      ROS_WARN("WAVE_PRED:KF_r loading failed, quitting");
    }
    for (size_t i = 0; i < STATE_COMPONENTS; i++) {
      if (!estiplan.getParam("q", q(i, i))) {
        ROS_WARN("WAVE_PRED:KF_q loading failed, quitting");
      }
    }
  }
  q_dash = 0.5 * ((phi * q * phi.transpose()) + q) * sample_time;
  p_k_dash = ((phi * p_k_dash * phi.transpose()) + q_dash).eval();
  l_k = p_k_dash * c_t.transpose() *
        (c_t * p_k_dash * c_t.transpose() + r).inverse();
  x_predicted = phi * x_t;
  w_k_hat = c_t * x_predicted;
  x_predicted = (x_predicted + (l_k * (w_k - w_k_hat))).eval();
  w_k_hat = c_t * x_predicted;
  p_k = (Eigen::MatrixXd::Identity(STATE_COMPONENTS, STATE_COMPONENTS) -
         (l_k * c_t)) *
        p_k_dash;
  x_t = x_predicted;
  p_k_dash = p_k;
  msg_time = ros::Time::now().toNSec();
  usv_msg.pose.pose.position.x = x_predicted(0);
  usv_msg.pose.pose.position.y = x_predicted(1);
  usv_msg.pose.pose.position.z = x_predicted(2);
  usv_msg.twist.twist.linear.x = x_predicted(3);
  usv_msg.twist.twist.linear.y = x_predicted(4);
  usv_msg.twist.twist.linear.z = x_predicted(5);
  usv_msg.header.stamp = ros::Time::now();
  wave_observer.publish(usv_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "usv_state_prediction",
            ros::init_options::AnonymousName);
  ros::NodeHandle estiplan("~");
  if (!estiplan.getParam("sampling_freq", sampling_freq)) {
    ROS_ERROR("sampling_freq loading failed");
    return -1;
  }
  sample_time = 1 / sampling_freq;
  phi.coeffRef(0, 3) = sample_time;
  phi.coeffRef(1, 4) = sample_time;
  phi.coeffRef(2, 5) = sample_time;
  if (!estiplan.getParam("t_future", t_future)) {
    ROS_WARN("WAVE_PRED:t_future loading failed, using 0.0 default");
  }
  if (!estiplan.getParam("r", r(0, 0))) {
    ROS_WARN("WAVE_PRED:KF_r loading failed, quitting");
  }
  if (!estiplan.getParam("r", r(1, 1))) {
    ROS_WARN("WAVE_PRED:KF_r loading failed, quitting");
  }
  if (!estiplan.getParam("r", r(2, 2))) {
    ROS_WARN("WAVE_PRED:KF_r loading failed, quitting");
  }
  if (!estiplan.getParam("online_tune", _online_tune_)) {
    ROS_WARN("WAVE_PRED:online_tune loading failed, default is false");
  }
  if (!estiplan.getParam("diagnostics", _diagnostics_)) {
    ROS_WARN("WAVE_PRED:diagnostics loading failed, default is false");
  }
  for (size_t i = 0; i < STATE_COMPONENTS; i++) {
    if (!estiplan.getParam("q", q(i, i))) {
      ROS_WARN("WAVE_PRED:KF_q loading failed, quitting");
      return -1;
    }
  }
  c_t.coeffRef(0, 0) = 1.0;
  c_t.coeffRef(1, 1) = 1.0;
  c_t.coeffRef(2, 2) = 1.0;
  // std::cout << c_t << std::endl;
  ros::Subscriber sub_pose =
      estiplan.subscribe("odometry_in", 1000, OdomCallback);
  wave_observer = estiplan.advertise<nav_msgs::Odometry>("observer", 1000);
  pose_pred_pub =
      estiplan.advertise<geometry_msgs::PoseArray>("pose_pred_out", 1000);
  // detected_output =
  //     estiplan.advertise<usv_estiplan::Fftarray>("idenitification", 1000);
  // ros::ServiceServer prediction_srv = estiplan.advertiseService(
  //     "/wave_prediction/" + dof_name, PredictionServiceCallback);
  // ros::Publisher prediction_publisher =
  //     estiplan.advertise<usv_estiplan::Wavefuture>("predictions", 1000);
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    if (ros::isShuttingDown()) {
    }
    geometry_msgs::PoseArray pose_pred_msg;
    x_predicted_pred = x_t;
    for (size_t i = 0; i < 100; i++) {
      q_dash_predict = 0.5 * ((phi * q * phi.transpose()) + q) * sample_time;
      p_k_dash_pred = ((phi * p_k_dash * phi.transpose()) + q_dash).eval();
      x_predicted_pred = phi * x_predicted_pred;
      temp_msg.position.x = x_predicted_pred(0);
      temp_msg.position.y = x_predicted_pred(1);
      temp_msg.position.z = x_predicted_pred(2);
      temp_msg.orientation.w = 1.0;
      pose_pred_msg.poses.push_back(temp_msg);
    }
    pose_pred_msg.header.stamp = ros::Time::now();
    pose_pred_msg.header.frame_id = "uav1/gps_origin";
    pose_pred_pub.publish(pose_pred_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}