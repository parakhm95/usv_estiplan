// #include <fftw3.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "usv_estiplan/Fftarray.h"
#include "usv_estiplan/Fftresult.h"
#include "usv_estiplan/PredictionOutput.h"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unsupported/Eigen/MatrixFunctions>
using namespace std;

#define REAL 0
#define IMAG 1

const int WAVE_COMPONENTS = 20;
int sampling_freq = 100;
float sample_time = 1 / sampling_freq;
uint64_t iter_global = 0;
usv_estiplan::Fftarray fft_array{};
double msg_time;
std_msgs::Float64 wave_msg;
std_msgs::Float64 wave_future_msg;
ros::Publisher wave_predictor;
ros::Publisher wave_future;
string odom_topic_name;
string dof_name;
Eigen::VectorXd w_k_hat(1);
Eigen::VectorXd w_k(1);
Eigen::Vector2d x_i;                                       // X(i,0) single mode
Eigen::VectorXd x_t(2 * (WAVE_COMPONENTS + 1), 1);         // X(t_0) mode vector
Eigen::VectorXd l_k(2 * (WAVE_COMPONENTS + 1), 1);         // L_k Kalman gain
Eigen::VectorXd x_predicted(2 * (WAVE_COMPONENTS + 1), 1); // X_k+1 predicted
// A(t_0)
Eigen::Matrix<double, 2 * (WAVE_COMPONENTS + 1), 2 * (WAVE_COMPONENTS + 1)>
    a_t = Eigen::MatrixXd::Zero(2 * (WAVE_COMPONENTS + 1),
                                2 * (WAVE_COMPONENTS + 1));
// Phi_0
Eigen::MatrixXd phi(2 * (WAVE_COMPONENTS + 1), 2 * (WAVE_COMPONENTS + 1));
// P_k+1 predicted
Eigen::Matrix<double, 2 * (WAVE_COMPONENTS + 1), 2 * (WAVE_COMPONENTS + 1)>
    p_k_dash = Eigen::MatrixXd::Zero(2 * (WAVE_COMPONENTS + 1),
                                     2 * (WAVE_COMPONENTS + 1));
// P_k+1 actual
Eigen::Matrix<double, 2 * (WAVE_COMPONENTS + 1), 2 * (WAVE_COMPONENTS + 1)>
    p_k = Eigen::MatrixXd::Zero(2 * (WAVE_COMPONENTS + 1),
                                2 * (WAVE_COMPONENTS + 1));

Eigen::MatrixXd c_t(1, 2 * (WAVE_COMPONENTS + 1)); // C(t_0)
Eigen::Matrix2d a_i;                               // A_i(t_0)
Eigen::VectorXd r(1);
Eigen::Matrix<double, 2 * (WAVE_COMPONENTS + 1), 2 * (WAVE_COMPONENTS + 1)>
    q_dash;
Eigen::Matrix<double, 2 * (WAVE_COMPONENTS + 1), 2 * (WAVE_COMPONENTS + 1)> q =
    Eigen::MatrixXd::Zero(2 * (WAVE_COMPONENTS + 1), 2 * (WAVE_COMPONENTS + 1));
bool first_start = true;
// row-0 is last, row-1 is current
// true is present, false is notpresent
bool parity_matrix[2][WAVE_COMPONENTS];
Eigen::VectorXd freq_components(WAVE_COMPONENTS);

Eigen::VectorXd phase(1);
Eigen::VectorXd freq(1);
Eigen::VectorXd amplitude(1);
// std::ofstream csv_debug;
Eigen::IOFormat CSVFormat(4, Eigen::DontAlignCols, ";", "\n");

bool PredictionServiceCallback(usv_estiplan::PredictionOutput::Request &req,
                               usv_estiplan::PredictionOutput::Response &res) {
  double pred_time = req.pred_time;
  for (size_t i = 0; i < WAVE_COMPONENTS; i++) {

    //------extracting frequency-----
    freq(0) = freq_components[i];

    // --------------------------extracting phase------------------------
    phase(0) = atan2(2 * M_PI * freq(0) * x_t(2 * i), x_t((2 * i) + 1)) -
               2 * M_PI * freq(0);

    // -------------------extracting amplitude-------------------------
    if (sin((2 * M_PI * freq(0)) + phase(0)) != 0.0) {
      amplitude(0) = x_t(2 * i) / sin((2 * M_PI * freq(0)) + phase(0));
    } else {
      amplitude(0) = 0.0;
    }
    // ------------ predictions ----------------
    res.zeroth = 0.0;
    res.first = 0.0;
    res.second = 0.0;
    {
      // --------------- zeroth ------------------
      res.zeroth +=
          amplitude(0) * sin((2 * M_PI * freq(0) * pred_time) + phase(0));
    }
    {
      // --------------- first --------------------
      res.first += amplitude(0) * 2 * M_PI * freq(0) *
                   cos((2 * M_PI * freq(0) * pred_time) + phase(0));
    }
    {
      // --------------- second --------------------
      res.second += -amplitude(0) * pow(2 * M_PI * freq(0), 2) *
                    sin((2 * M_PI * freq(0) * pred_time) + phase(0));
    }
  }
  // ------------------- adding random noise component -----------------
  res.zeroth += x_t(2 * WAVE_COMPONENTS);
  res.response = "success";
  return true;
}

void OdomCallback(const geometry_msgs::PoseStamped &msg) {
  tf2::Quaternion quat(msg.pose.orientation.x, msg.pose.orientation.y,
                       msg.pose.orientation.z, msg.pose.orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  if (dof_name == "x") {
    w_k(0) = msg.pose.position.x;
  } else if (dof_name == "y") {
    w_k(0) = msg.pose.position.y;
  } else if (dof_name == "z") {
    w_k(0) = msg.pose.position.z;
  } else if (dof_name == "roll") {
    w_k(0) = roll;
  } else if (dof_name == "pitch") {
    w_k(0) = pitch;
  } else if (dof_name == "yaw") {
    w_k(0) = yaw;
  } else {
    ROS_ERROR("dof_name was not specified, shutting down");
    ros::shutdown();
  }
  if (!first_start) {
    x_predicted = phi * x_t;
    q_dash = 0.5 * ((phi * q * phi.transpose()) + q) * sample_time;
    p_k_dash = ((phi * p_k_dash * phi.transpose()) + q_dash).eval();
    l_k = p_k_dash * c_t.transpose() *
          (c_t * p_k_dash * c_t.transpose() + r).inverse();
    w_k_hat = c_t * x_t;
    x_predicted = (x_predicted + (l_k * (w_k - w_k_hat))).eval();
    w_k_hat = c_t * x_predicted;
    p_k = (Eigen::MatrixXd::Identity(2 * (WAVE_COMPONENTS + 1),
                                     2 * (WAVE_COMPONENTS + 1)) -
           (l_k * c_t)) *
          p_k_dash;
    x_t = x_predicted;
    p_k_dash = p_k;
    wave_msg.data = w_k_hat(0);
    wave_future_msg.data = 0.0;
    wave_predictor.publish(wave_msg);
    wave_future.publish(wave_future_msg);
  }
}

void FftCallback(usv_estiplan::Fftresult in_msg) {
  if (dof_name == "x") {
    fft_array = in_msg.x;
  } else if (dof_name == "y") {
    fft_array = in_msg.y;
  } else if (dof_name == "z") {
    fft_array = in_msg.z;
  } else if (dof_name == "roll") {
    fft_array = in_msg.roll;
  } else if (dof_name == "pitch") {
    fft_array = in_msg.pitch;
  } else if (dof_name == "yaw") {
    fft_array = in_msg.yaw;
  } else {
    ROS_ERROR("dof_name was not specified, shutting down");
    ros::shutdown();
  }
  first_start = false;
  // reset parity matrix
  for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
    parity_matrix[0][i] = false;
    parity_matrix[1][i] = false;
  }

  // check which components already exist
  for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
    for (size_t j = 0; j < WAVE_COMPONENTS; j++) {
      if (trunc(1000. * fft_array.frequency[i]) ==
              trunc(1000. * freq_components(j)) &&
          (parity_matrix[0][j] != true) && (parity_matrix[1][i] != true)) {
        parity_matrix[0][j] = true; // row-0 for last_msg
        parity_matrix[1][i] = true; // row-1 for new_msg
      }
    }
  }
  size_t spot_avlb = 0;
  for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
    if (!parity_matrix[0][i]) {
      while (parity_matrix[1][spot_avlb] && spot_avlb < WAVE_COMPONENTS) {
        spot_avlb += 1;
      }
      freq_components(i) = fft_array.frequency[spot_avlb];
      // changing A matrices
      a_i(1) = -pow(2 * M_PI * fft_array.frequency[spot_avlb], 2);
      a_t.block(2 * i, 2 * i, 2, 2) = a_i;
      // changing X matrices
      x_i(0) = fft_array.amplitude[spot_avlb] * sin(fft_array.phase[spot_avlb]);
      x_i(1) = 2 * M_PI * fft_array.amplitude[spot_avlb] *
               fft_array.frequency[spot_avlb] * cos(fft_array.phase[spot_avlb]);
      std::cout << x_i(0) << std::endl;
      std::cout << x_i(1) << std::endl;
      x_t.block(2 * i, 0, 2, 1) = x_i;
      spot_avlb += 1;
    }
  }
  // updating Phi
  phi = (a_t * sample_time).exp();
  msg_time = ros::Time::now().toNSec();
}

int main(int argc, char **argv) {

  r(0) = 0.05;
  x_t(2 * WAVE_COMPONENTS) = 0.0;
  x_t(2 * WAVE_COMPONENTS + 1) = 0.0;
  a_i(0) = 0.0;
  a_i(2) = 1.0;
  a_i(3) = 0.0;
  p_k(2 * WAVE_COMPONENTS + 1, 2 * WAVE_COMPONENTS + 1) = 10.0;
  for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
    q(i, i) = 0.01;
  }
  q(2 * WAVE_COMPONENTS, 2 * WAVE_COMPONENTS) = 0.001;
  q((2 * WAVE_COMPONENTS) + 1, (2 * WAVE_COMPONENTS) + 1) = 0.001;

  for (size_t i = 0; i < WAVE_COMPONENTS + 1; i++) {
    c_t(2 * i) = 1.0;
    c_t((2 * i) + 1) = 0.0;
  }
  ros::init(argc, argv, "wave_prediction", ros::init_options::AnonymousName);
  ros::NodeHandle estiplan("~");
  if (argc != 2) {
    ROS_ERROR("Specify the DOF arg in launch file");
    return -1;
  }
  dof_name = argv[1];
  if (!estiplan.getParam("/topics/sampling_freq", sampling_freq)) {
    sample_time = 1 / sampling_freq;
    ROS_ERROR("sampling_freq loading failed");
    return -1;
  }
  if (!estiplan.getParam("/topics/odom", odom_topic_name)) {
    ROS_ERROR("WAVE_PRED:odom_topic loading failed");
    return -1;
  }

  ros::Subscriber sub = estiplan.subscribe("/fft_output/", 1000, FftCallback);
  ros::Subscriber sub_pose =
      estiplan.subscribe(odom_topic_name, 1000, OdomCallback);
  wave_predictor = estiplan.advertise<std_msgs::Float64>(
      "/wave_prediction/" + dof_name, 1000);
  wave_future = estiplan.advertise<std_msgs::Float64>(
      "/wave_future_5s/" + dof_name, 1000);
  ros::ServiceServer prediction_srv = estiplan.advertiseService(
      "/wave_prediction/" + dof_name, PredictionServiceCallback);
  while (ros::ok()) {
    if (ros::isShuttingDown()) {
    }
    ros::spin();
  }

  return 0;
}