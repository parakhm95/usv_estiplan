// #include <fftw3.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <random>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

#define REAL 0
#define IMAG 1

const int wave_components = 10;
const float CONSTANT_OFFSET_X = 13.2;
const float CONSTANT_OFFSET_Y = 40.2;
const float CONSTANT_OFFSET_Z = 9.2;
double roll = 0.0;
double pitch = 0.0;
double yaw = 0.0;
uint64_t iter_global = 0;
string odom_topic_name;
geometry_msgs::PoseStamped pose_msg{};
geometry_msgs::Vector3 rpy_msg{};
double msg_time;
double one[wave_components] = {0.1,  0.2,  0.15, 0.7,  0.65,
                               0.35, 0.23, 0.5,  0.05, 0.85};
double two[wave_components] = {0.1,  0.4,  0.25, 0.31, 0.05,
                               0.35, 0.13, 0.04, 0.48, 0.6};
double three[wave_components] = {0.1, 0.2, 0.3, 0.4, 0.5,
                                 0.6, 0.7, 0.8, 0.9, 1.0};

int main(int argc, char **argv) {

  ros::init(argc, argv, "wave_generator");
  ros::NodeHandle estiplan("~");
  if (!estiplan.getParam("/topics/odom", odom_topic_name)) {
    ROS_ERROR("WAVE_GEN:odom_topic loading failed");
    return -1;
  }
  ros::Publisher wave_generator =
      estiplan.advertise<geometry_msgs::PoseStamped>(odom_topic_name, 1000);
    ros::Publisher rpy_generator =
      estiplan.advertise<geometry_msgs::Vector3>(odom_topic_name+"/rpy", 1000);
  ros::Rate loop_rate(100.0);
  double wave_output;
  double time_elap = 0;
  while (ros::ok()) {
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.position.y = 0.0;
    pose_msg.pose.position.z = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    time_elap = ros::Time::now().toNSec() * 1e-9;
    for (size_t i = 0; i < wave_components; i++) {
      pose_msg.pose.position.x +=
          one[i] * sin((two[i] * 2 * M_PI * time_elap) + three[i]);
      pose_msg.pose.position.y +=
          one[i] * sin((three[i] * 2 * M_PI * time_elap) + two[i]);
      pose_msg.pose.position.z +=
          two[i] * sin((one[i] * 2 * M_PI * time_elap) + three[i]);
      roll += two[i] * sin((three[i] * 2 * M_PI * time_elap) + one[i]);
      pitch += three[i] * sin((one[i] * 2 * M_PI * time_elap) + two[i]);
      yaw += three[i] * sin((two[i] * 2 * M_PI * time_elap) + one[i]);
    }
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(-0.3, 0.3);
    std::uniform_real_distribution<double> dist_small(-0.05, 0.05);
    pose_msg.pose.position.x += dist(mt) + CONSTANT_OFFSET_X;
    pose_msg.pose.position.y += dist(mt) + CONSTANT_OFFSET_Y;
    pose_msg.pose.position.z += dist(mt) + CONSTANT_OFFSET_Z;
    roll = roll/3 + dist_small(mt);
    pitch = pitch/3 + dist_small(mt);
    yaw = yaw/3 + dist_small(mt);
    rpy_msg.x = roll;
    rpy_msg.y = pitch;
    rpy_msg.z = yaw;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(
        roll, pitch,
        yaw); // Create this quaternion from roll/pitch/yaw (in radians)
        myQuaternion.normalize();
    pose_msg.pose.orientation.x = myQuaternion[0];
    pose_msg.pose.orientation.y = myQuaternion[1];
    pose_msg.pose.orientation.z = myQuaternion[2];
    pose_msg.pose.orientation.w = myQuaternion[3];

    wave_generator.publish(pose_msg);
    rpy_generator.publish(rpy_msg);
    if (ros::isShuttingDown()) {
      // fft_output_capt.close();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}