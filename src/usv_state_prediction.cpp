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

// std::ofstream csv_debug;
LinearModel model1;
LinearInputModel model2;

void OdomCallback(const geometry_msgs::PoseStamped &msg) {
  ROS_INFO("[Tag message received]");
  tag_received = true;
  model1.updateModel(msg);
  model2.updateModel(msg);
  model1.getPrediction(temp_msg, 0.00);
  model2.getPrediction(temp_msg, 0.00);
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
  pose_pred_pub_model2 = estiplan.advertise<geometry_msgs::PoseArray>(
      "model2_pose_predictions_out", 1000);
  pub_last_pose = estiplan.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "last_predicted_pose", 1000);
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
    model2.iterateModel();
    geometry_msgs::PoseArray pose_pred_msg;
    geometry_msgs::PoseArray model2_pose_pred_msg;
    geometry_msgs::PoseWithCovarianceStamped msg_last_pose_with_covariance;
    // TODO: Remove the hardcoded numbers for prediction timestep and horizons
    for (double i = 0.0; i < 4.0; i += 0.1) {
      model1.getPrediction(temp_msg, i);
      //  model2.getPrediction(model2_temp_msg, i);
      temp_msg.position.z += 6.0;
      // model2_temp_msg.position.z += 6.0;
      pose_pred_msg.poses.push_back(temp_msg);
      // model2_pose_pred_msg.poses.push_back(model2_temp_msg);
      //   INVESTIGATE: WHY DOES EQUAL SIGN NOT WORK HERE
      if (i >= 3.9) {
        msg_last_pose_with_covariance.pose.pose = temp_msg;
      }
    }
    model2.returnPredictions(4.0, model2_pose_pred_msg);
    Eigen::MatrixXd last_covariance = model1.getCovarianceOfPrediction(4.0);
    for (int i = 0; i < 36; i++) {
      // Because ros covariance is row major, we transpose the column-major of
      // Eigen to access in the correct format
      msg_last_pose_with_covariance.pose.covariance[i] =
          last_covariance.transpose()(i);
    }
    msg_last_pose_with_covariance.header.stamp =
        ros::Time::now() + ros::Duration(3.9);
    // TODO remove hardcoded frame_id
    msg_last_pose_with_covariance.header.frame_id = "uav1/gps_origin";
    pose_pred_msg.header.stamp = ros::Time::now();
    pose_pred_msg.header.frame_id = "uav1/gps_origin";
    model2_pose_pred_msg.header.stamp = ros::Time::now();
    model2_pose_pred_msg.header.frame_id = "uav1/gps_origin";
    if (tag_received) {
      pose_pred_pub_model1.publish(pose_pred_msg);
      pose_pred_pub_model2.publish(model2_pose_pred_msg);
    }
    pub_last_pose.publish(msg_last_pose_with_covariance);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
