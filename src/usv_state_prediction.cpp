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
using namespace std;

#define REAL 0
#define IMAG 1

geometry_msgs::Pose temp_msg;
ros::Publisher wave_observer_model1;
ros::Publisher wave_observer_model2;
ros::Publisher pose_pred_pub_model1;
ros::Publisher pose_pred_pub_model2;
double last_update_time;
bool tag_received = false;

// std::ofstream csv_debug;
LinearModel model1;

void OdomCallback(const geometry_msgs::PoseStamped &msg) {
  ROS_INFO("[Tag message received]");
  tag_received = true;
  model1.updateModel(msg);
  model1.getPrediction(temp_msg, 0.00);
  wave_observer_model1.publish(temp_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "usv_state_prediction",
            ros::init_options::AnonymousName);
  ros::NodeHandle estiplan("~");
  model1.initialiseModel(estiplan);
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
