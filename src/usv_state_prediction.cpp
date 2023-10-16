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
#include <usv_linear_input_model.hpp>
#include <usv_linear_model.hpp>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "usv_estiplan/OdometryArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
using namespace std;

#define REAL 0
#define IMAG 1

geometry_msgs::Pose temp_msg;
geometry_msgs::Pose model2_temp_msg;
ros::Publisher wave_observer_model1;
ros::Publisher wave_observer_model2;
ros::Publisher pose_pred_pub_model1;
ros::Publisher pose_pred_pub_model2;
ros::Publisher pub_last_pose;
double last_update_time;
bool tag_received = false;
bool horizon_loaded = false;
bool prediction_timestep_loaded = false;
int _lmpc_horizon_ = 0;
double _lmpc_prediction_timestep_ = 0.0;
string _uav_frame_ = "uav1/gps_origin";

// std::ofstream csv_debug;
LinearModel model1;
LinearInputModel model2;

void OdomCallback(const geometry_msgs::PoseStamped &msg) {
  ROS_INFO("[Tag message received]");
  tag_received = true;
  _uav_frame_ = msg.header.frame_id;
  model1.updateModel(msg);
  model2.correctModel(msg);
  model1.getPrediction(temp_msg, 0.00);
  model2.getCorrectedState(temp_msg);
  wave_observer_model1.publish(temp_msg);
  wave_observer_model2.publish(temp_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "usv_state_prediction",
            ros::init_options::AnonymousName);
  ros::NodeHandle estiplan("~");
  model1.initialiseModel(estiplan);
  model2.initialiseModel(estiplan);
  // std::cout << c_t << std::endl;
  ros::Subscriber sub_pose =
      estiplan.subscribe("odometry_in", 1000, OdomCallback);
  wave_observer_model1 =
      estiplan.advertise<geometry_msgs::Pose>("observer_model1", 1000);
  wave_observer_model2 =
      estiplan.advertise<geometry_msgs::Pose>("observer_model2", 1000);
  pose_pred_pub_model1 = estiplan.advertise<geometry_msgs::PoseArray>(
      "model1_pose_predictions_out", 1000);
  pose_pred_pub_model2 = estiplan.advertise<usv_estiplan::OdometryArray>(
      "model2_pose_predictions_out", 1000);
  pub_last_pose = estiplan.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "last_predicted_pose", 1000);
  // detected_output =
  //     estiplan.advertise<usv_estiplan::Fftarray>("idenitification", 1000);
  // ros::ServiceServer prediction_srv = estiplan.advertiseService(
  //     "/wave_prediction/" + dof_name, PredictionServiceCallback);
  // ros::Publisher prediction_publisher =
  //     estiplan.advertise<usv_estiplan::Wavefuture>("predictions", 1000);
  // if (!estiplan.getParam("uav_name", _uav_name_)) {
  //  ROS_ERROR("[LinearModel] : Couldn't load uav_name, using the default");
  //}
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    if (ros::isShuttingDown()) {
    }
    model2.iterateModel();
    geometry_msgs::PoseArray pose_pred_msg;
    usv_estiplan::OdometryArray model2_pose_pred_msg;
    geometry_msgs::PoseWithCovarianceStamped msg_last_pose_with_covariance;
    if (!horizon_loaded) {
      if (!estiplan.getParam("/lmpc_pred_horizon", _lmpc_horizon_)) {
        ROS_ERROR("[LinearModel] : Couldn't load lmpc_pred_horizon");
      } else {
        horizon_loaded = true;
      }
    }
    if (!prediction_timestep_loaded) {
      if (!estiplan.getParam("/lmpc_pred_timestep",
                             _lmpc_prediction_timestep_)) {
        ROS_ERROR("[LinearModel] : Couldn't load lmpc_pred_timestep");
      } else {
        prediction_timestep_loaded = true;
      }
    }
    if (horizon_loaded and prediction_timestep_loaded) {

      for (double i = 0.0; i < 4.0; i += 0.1) {
      model1.getPrediction(temp_msg, i);
      pose_pred_msg.poses.push_back(temp_msg);
      }
      double total_horizon_time = _lmpc_prediction_timestep_ * _lmpc_horizon_;
      model2.returnPredictions(total_horizon_time, model2_pose_pred_msg,
                               _lmpc_prediction_timestep_);
      msg_last_pose_with_covariance.pose.pose =
          model2_pose_pred_msg.odom_array[_lmpc_horizon_ - 1].pose;
      Eigen::MatrixXd last_covariance =
          model2.getCovarianceOfPrediction(total_horizon_time);
      for (int i = 0; i < 36; i++) {
        // Because ros covariance is row major, we transpose the column-major of
        // Eigen to access in the correct format
        msg_last_pose_with_covariance.pose.covariance[i] =
            last_covariance.transpose()(i);
      }
      msg_last_pose_with_covariance.header.stamp =
          ros::Time::now() + ros::Duration(total_horizon_time);
      msg_last_pose_with_covariance.header.frame_id = _uav_frame_;
      pose_pred_msg.header.stamp = ros::Time::now();
      pose_pred_msg.header.frame_id = _uav_frame_;
      model2_pose_pred_msg.header.stamp = ros::Time::now();
      model2_pose_pred_msg.header.frame_id = _uav_frame_;
      if (tag_received) {
        pose_pred_pub_model1.publish(pose_pred_msg);
        pose_pred_pub_model2.publish(model2_pose_pred_msg);
      }
      pub_last_pose.publish(msg_last_pose_with_covariance);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
