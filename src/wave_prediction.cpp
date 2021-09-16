// #include <fftw3.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "usv_estiplan/Fftresult.h"
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
usv_estiplan::Fftresult fft_msg{};
double msg_time;
std_msgs::Float64 wave_msg;
std_msgs::Float64 wave_future_msg;
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
bool parity_matrix[2][WAVE_COMPONENTS];
Eigen::VectorXd freq_components(WAVE_COMPONENTS);

Eigen::VectorXd phase(1);
Eigen::VectorXd freq(1);
Eigen::VectorXd amplitude(1);
// std::ofstream csv_debug;
Eigen::IOFormat CSVFormat(4, Eigen::DontAlignCols, ";", "\n");

// row-0 is last, row-1 is current
// true is present, false is notpresent

void OdomCallback(const geometry_msgs::PoseStamped &msg) {
  tf2::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
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
}

void FftCallback(usv_estiplan::Fftresult in_msg) {
  // if (first_start) {
  //   csv_debug.open(
  //       "/home/octo/Documents/offshore_wrs/src/usv_estiplan/csv_debug.csv");
  //   csv_debug << "freq_mat;parity0;cur_msg;parity1;";
  //   csv_debug << "\n";
  // } else {
  //   csv_debug.open(
  //       "/home/octo/Documents/offshore_wrs/src/usv_estiplan/csv_debug.csv",
  //       std::ios::app);
  // }
  fft_msg = in_msg;
  first_start = false;
  // reset parity matrix
  for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
    parity_matrix[0][i] = false;
    parity_matrix[1][i] = false;
  }

  // check which components already exist
  for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
    for (size_t j = 0; j < WAVE_COMPONENTS; j++) {
      if (trunc(1000. * fft_msg.x.frequency[i]) ==
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
      freq_components(i) = fft_msg.x.frequency[spot_avlb];
      // changing A matrices
      a_i(1) = -pow(2 * M_PI * fft_msg.x.frequency[spot_avlb], 2);
      a_t.block(2 * i, 2 * i, 2, 2) = a_i;
      // changing X matrices
      x_i(0) = fft_msg.x.amplitude[spot_avlb] * sin(fft_msg.x.phase[spot_avlb]);
      x_i(1) = 2 * M_PI * fft_msg.x.amplitude[spot_avlb] *
               fft_msg.x.frequency[spot_avlb] * cos(fft_msg.x.phase[spot_avlb]);
      std::cout << x_i(0) << std::endl;
      std::cout << x_i(1) << std::endl;
      x_t.block(2 * i, 0, 2, 1) = x_i;
      spot_avlb += 1;
    }
  }
  // updating Phi
  phi = (a_t * sample_time).exp();
  msg_time = ros::Time::now().toNSec();
  // for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
  //   csv_debug << freq_components[i] << ";" << parity_matrix[0][i] << ";"
  //             << in_msg.frequency[i] << ";" << parity_matrix[1][i] << ";"
  //             << "\n";
  // }
  // csv_debug.close();
  // }
}

int main(int argc, char **argv) {
  // csv_debug.open(
  // "/home/octo/Documents/offshore_wrs/src/usv_estiplan/csv_debug.csv");

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
    ROS_WARN("Specify the DOF arg in launch file");
    return -1;
  }
  dof_name = argv[1];
  if (!estiplan.getParam("/topics/sampling_freq", sampling_freq)) {
    sample_time = 1 / sampling_freq;
    ROS_WARN("sampling_freq loading failed");
    return -1;
  }
  ROS_INFO("Are we here1?");
  if (!estiplan.getParam("/topics/odom", odom_topic_name)) {
    ROS_WARN("WAVE_PRED:odom_topic loading failed");
    return -1;
  }

  ros::Subscriber sub = estiplan.subscribe("/fft_output/", 1000, FftCallback);
  ros::Subscriber sub_pose =
      estiplan.subscribe(odom_topic_name, 1000, OdomCallback);
  ros::Publisher wave_predictor = estiplan.advertise<std_msgs::Float64>(
      "/wave_prediction/" + dof_name, 1000);
  ros::Publisher wave_future = estiplan.advertise<std_msgs::Float64>(
      "/wave_future_5s/" + dof_name, 1000);
  ros::Rate loop_rate(sampling_freq);
  double wave_output;
  double time_elap = 0;
  while (ros::ok()) {
    if (!first_start) {

      // csv_debug << "freq_components" << std::endl;
      // csv_debug << freq_components.format(CSVFormat) << std::endl;
      // csv_debug << "x_t" << std::endl;
      // csv_debug << x_t.format(CSVFormat) << std::endl;
      x_predicted = phi * x_t;
      // csv_debug << "phi" << std::endl;
      // csv_debug << phi.format(CSVFormat) << std::endl;
      // csv_debug << "x_predicted";
      // csv_debug << "\n";

      // csv_debug << x_predicted.format(CSVFormat);
      // csv_debug << "\n";
      q_dash = 0.5 * ((phi * q * phi.transpose()) + q) * sample_time;
      // csv_debug << "q_dash";
      // csv_debug << "\n";

      // csv_debug << q_dash.format(CSVFormat);
      // csv_debug << "\n";
      p_k_dash = ((phi * p_k_dash * phi.transpose()) + q_dash).eval();
      // csv_debug << "p_k_dash";
      // csv_debug << "\n";
      // csv_debug << p_k_dash.format(CSVFormat);

      // csv_debug << "\n";
      l_k = p_k_dash * c_t.transpose() *
            (c_t * p_k_dash * c_t.transpose() + r).inverse();
      // csv_debug << "p_k_dash*c_t" << std::endl;
      // csv_debug << (p_k_dash * c_t.transpose()).format(CSVFormat) <<
      // std::endl; csv_debug << "(c_t * p_k_dash * c_t.transpose() +
      // r).inverse()"
      // << std::endl;
      // csv_debug << (c_t * p_k_dash * c_t.transpose() + r) << std::endl;
      // csv_debug << (c_t * p_k_dash * c_t.transpose() + r).inverse()
      // << std::endl;
      // csv_debug << "l_k";
      // csv_debug << "\n";

      // csv_debug << l_k.format(CSVFormat);
      // csv_debug << "\n";
      w_k_hat = c_t * x_t;
      x_predicted = (x_predicted + (l_k * (w_k - w_k_hat))).eval();
      w_k_hat = c_t * x_predicted;
      p_k = (Eigen::MatrixXd::Identity(2 * (WAVE_COMPONENTS + 1),
                                       2 * (WAVE_COMPONENTS + 1)) -
             (l_k * c_t)) *
            p_k_dash;
      // csv_debug << "p_k";
      // csv_debug << "\n";
      // csv_debug << p_k.format(CSVFormat);
      // csv_debug << "\n";
      x_t = x_predicted;
      p_k_dash = p_k;
      wave_output = 0;
      for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
        time_elap = (ros::Time::now().toNSec() - msg_time) * 1e-9;
        wave_output += fft_msg.x.amplitude[i] *
                       sin((fft_msg.x.frequency[i] * 2 * M_PI * time_elap) +
                           fft_msg.x.phase[i]);
      }
      wave_msg.data = w_k_hat(0);
      wave_predictor.publish(wave_msg);
      wave_future_msg.data = 0.0;
      for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
        freq(0) = freq_components[i];
        phase(0) = atan2(2 * M_PI * freq(0) * x_t(2 * i), x_t((2 * i) + 1)) -
                   2 * M_PI * freq(0);
        // time_ahead = (ros::Time::now().toNSec() * 1e-9) + 5.0;
        if (sin((2 * M_PI * freq(0)) + phase(0)) != 0.0) {
          amplitude(0) = x_t(2 * i) / sin((2 * M_PI * freq(0)) + phase(0));

        } else {
          amplitude(0) = 0.0;
        }
        wave_future_msg.data +=
            amplitude(0) * sin((2 * M_PI * freq(0) * 5) + phase(0));
      }
      wave_future_msg.data += x_t(2 * WAVE_COMPONENTS);
      wave_future.publish(wave_future_msg);
    }
    // std::cout << "Freq_components : " << std::endl;
    for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
      // std::cout << freq_components[i] << std::endl;
    }

    if (ros::isShuttingDown()) {
      // csv_debug.close();
    }
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}