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
#include <usv_linear_model.hpp>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
using namespace std;

#define REAL 0
#define IMAG 1

const int STATE_COMPONENTS = 8;
const int OUTPUT_COMPONENTS = 3;
geometry_msgs::Pose temp_msg;
ros::Publisher wave_observer_model1;
ros::Publisher wave_observer_model2;
ros::Publisher pose_pred_pub_model1;
ros::Publisher pose_pred_pub_model2;
double last_update_time;
bool tag_received = false;
Eigen::Matrix<double, STATE_COMPONENTS, 1> x_predicted_pred;  // X_k+1 predicted
// Phi_0
Eigen::Matrix<double, STATE_COMPONENTS, STATE_COMPONENTS> p_k_dash_pred =
    Eigen::MatrixXd::Identity(STATE_COMPONENTS, STATE_COMPONENTS);
// P_k+1 actual
Eigen::Matrix<double, STATE_COMPONENTS, STATE_COMPONENTS> q_dash_predict;
// row-0 is last, row-1 is current
// true is present, false is notpresent

// std::ofstream csv_debug;
LinearModel model1;

void OdomCallback(const geometry_msgs::PoseStamped &msg) {
  tag_received = true;

  model1.updateModel(msg);
  model1.getPrediction(temp_msg);
  wave_observer_model1.publish(temp_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "usv_state_prediction",
            ros::init_options::AnonymousName);
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
      return -1;
    }
  }
  // std::cout << c_t << std::endl;
  ros::Subscriber sub_pose =
      estiplan.subscribe("odometry_in", 1000, OdomCallback);
  wave_observer_model1 =
      estiplan.advertise<geometry_msgs::Pose>("observer_model1", 1000);
  wave_observer_model2 =
      estiplan.advertise<geometry_msgs::Pose>("observer_model2", 1000);
  pose_pred_pub_model1 = estiplan.advertise<geometry_msgs::PoseArray>(
      "model1_pose_predictions_out", 1000);
  pose_pred_pub_model2 = estiplan.advertise<geometry_msgs::PoseArray>(
      "model2_pose_predictions_out", 1000);
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
    for (double i = 0.0; i < 1.0; i += 0.01) {
      model1.getPrediction(temp_msg, i);
      pose_pred_msg.poses.push_back(temp_msg);
    }
    pose_pred_msg.header.stamp = ros::Time::now();
    pose_pred_msg.header.frame_id = "uav1/gps_origin";
    if (tag_received && model1.getCovarianceOfVxy() < 0.05) {
      pose_pred_pub_model1.publish(pose_pred_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
