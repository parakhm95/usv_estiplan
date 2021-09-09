// #include <fftw3.h>
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "usv_estiplan/Fftoutput.h"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;

#define REAL 0
#define IMAG 1

const int WAVE_COMPONENTS = 20;
const float SAMPLE_TIME = 0.01;
uint64_t iter_global = 0;
usv_estiplan::Fftoutput fft_msg{};
double msg_time;
std_msgs::Float64 wave_msg;
std_msgs::Float64 wave_future_msg;
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
double freq_components[WAVE_COMPONENTS];

Eigen::VectorXd phase(1);
Eigen::VectorXd freq(1);
Eigen::VectorXd amplitude(1);
std::ofstream csv_debug;

// row-0 is last, row-1 is current
// true is present, false is notpresent

void ImuCallback(sensor_msgs::Imu imu_data) {
  w_k(0) = imu_data.angular_velocity.x;
}

void FftCallback(usv_estiplan::Fftoutput in_msg) {
  if (first_start) {
    csv_debug.open(
        "/home/octo/Documents/offshore_wrs/src/usv_estiplan/csv_debug.csv");
    csv_debug << "freq_mat;parity0;cur_msg;parity1;";
    csv_debug << "\n";
  } else {
    csv_debug.open(
        "/home/octo/Documents/offshore_wrs/src/usv_estiplan/csv_debug.csv",
        std::ios::app);
  }
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
      if (trunc(1000. * fft_msg.frequency[i]) ==
              trunc(1000. * freq_components[j]) &&
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
      freq_components[i] = fft_msg.frequency[spot_avlb];
      // changing A matrices
      a_i(1) = -pow(2 * M_PI * fft_msg.frequency[spot_avlb], 2);
      a_t.block(2 * i, 2 * i, 2, 2) = a_i;
      // changing X matrices
      x_i(0) = fft_msg.amplitude[spot_avlb] * sin(fft_msg.phase[spot_avlb]);
      x_i(1) = 2 * M_PI * fft_msg.amplitude[spot_avlb] *
               fft_msg.frequency[spot_avlb] * cos(fft_msg.phase[spot_avlb]);
      x_t.block(2 * i, 0, 2, 1) = x_i;
      spot_avlb += 1;
    }
  }
  // updating Phi
  phi = (a_t * SAMPLE_TIME).exp();
  msg_time = ros::Time::now().toNSec();
  for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
    csv_debug << freq_components[i] << ";" << parity_matrix[0][i] << ";"
              << in_msg.frequency[i] << ";" << parity_matrix[1][i] << ";"
              << "\n";
  }
  csv_debug.close();
}

int main(int argc, char **argv) {
  r(0) = 0.11;
  x_t(2 * WAVE_COMPONENTS) = 0.0;
  x_t(2 * WAVE_COMPONENTS + 1) = 0.0;
  a_i(0) = 0.0;
  a_i(2) = 1.0;
  a_i(3) = 0.0;
  q(0, 0) = 0.1;
  q(1, 1) = 0.1;
  q(2, 2) = 0.05;
  q(3, 3) = 0.05;
  q(4, 4) = 0.05;
  q(5, 5) = 0.05;
  for (size_t i = 0; i < WAVE_COMPONENTS + 1; i++) {
    c_t(2 * i) = 1.0;
    c_t((2 * i) + 1) = 0.0;
  }
  ros::init(argc, argv, "wave_prediction");
  ros::NodeHandle estiplan;
  ros::Subscriber sub = estiplan.subscribe("/fft_output/", 1000, FftCallback);
  ros::Subscriber sub_imu = estiplan.subscribe("/wamv/imu", 1000, ImuCallback);
  ros::Publisher wave_predictor =
      estiplan.advertise<std_msgs::Float64>("/wave_prediction", 1000);
  ros::Publisher wave_future =
      estiplan.advertise<std_msgs::Float64>("/wave_future_5s", 1000);
  ros::Rate loop_rate(1 / SAMPLE_TIME);
  double wave_output;
  double time_elap = 0;
  while (ros::ok()) {
    if (!first_start) {
      x_predicted = phi * x_t;
      q_dash = 0.5 * (phi * q * phi.transpose() + q) * SAMPLE_TIME;
      p_k_dash = (phi * p_k_dash * phi.transpose() + q_dash).eval();
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
      wave_output = 0;
      for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
        time_elap = (ros::Time::now().toNSec() - msg_time) * 1e-9;
        wave_output += fft_msg.amplitude[i] *
                       sin((fft_msg.frequency[i] * 2 * M_PI * time_elap) +
                           fft_msg.phase[i]);
      }
      wave_msg.data = w_k_hat(0);
      wave_predictor.publish(wave_msg);
      wave_future_msg.data = 0.0;
      for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
        freq(0) = freq_components[i];
        phase(0) = atan2(2 * M_PI * freq(0) * x_t(2 * i), x_t((2 * i) + 1)) -
                   2 * M_PI * freq(0);
        if (sin((2 * M_PI * freq(0)) + phase(0)) != 0.0) {
          amplitude(0) = x_t(2 * i) / sin((2 * M_PI * freq(0)) + phase(0));

        } else {
          amplitude(0) = 0.0;
        }
        std::cout << "amplitude :" << amplitude(0) << std::endl;
        wave_future_msg.data += amplitude(0) * sin(phase(0));
      }
      wave_future.publish(wave_future_msg);
    }
    // std::cout << "Freq_components : " << std::endl;
    for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
      // std::cout << freq_components[i] << std::endl;
    }

    if (ros::isShuttingDown()) {
    }
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}