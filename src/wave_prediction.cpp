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

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "usv_estiplan/Fftarray.h"
#include "usv_estiplan/Fftresult.h"
#include "usv_estiplan/Float64Stamped.h"
#include "usv_estiplan/PredictionOutput.h"
#include "usv_estiplan/Wavefuture.h"
using namespace std;

#define REAL 0
#define IMAG 1

const int WAVE_COMPONENTS = 20;
double sampling_freq = 100;
double sample_time = 1 / sampling_freq;
uint64_t iter_global = 0;
usv_estiplan::Fftarray fft_array{};
usv_estiplan::Fftarray ident_msg{};
double msg_time;
usv_estiplan::Float64Stamped wave_msg;
usv_estiplan::Float64Stamped decoded_msg;
usv_estiplan::Wavefuture wave_future_msg;
ros::Publisher wave_observer;
ros::Publisher wave_decoded;
ros::Publisher wave_future;
string dof_name;
double t_future = 0.0;
double pred_horizon = 0.0;
bool horizon_loaded = false;
bool _diagnostics_ = false;
bool _online_tune_ = true;
Eigen::VectorXd w_k_hat(1);
Eigen::VectorXd w_k(1);
Eigen::Vector2d x_i;                                // X(i,0) single mode
Eigen::VectorXd x_t(2 * (WAVE_COMPONENTS + 1), 1);  // X(t_0) mode vector
Eigen::VectorXd l_k(2 * (WAVE_COMPONENTS + 1), 1);  // L_k Kalman gain
Eigen::VectorXd x_predicted(2 * (WAVE_COMPONENTS + 1), 1);  // X_k+1 predicted
// A(t_0)
Eigen::Matrix<double, 2 * (WAVE_COMPONENTS + 1), 2 * (WAVE_COMPONENTS + 1)>
    a_t = Eigen::MatrixXd::Zero(2 * (WAVE_COMPONENTS + 1),
                                2 * (WAVE_COMPONENTS + 1));
// Phi_0
Eigen::MatrixXd phi(2 * (WAVE_COMPONENTS + 1), 2 * (WAVE_COMPONENTS + 1));
// P_k+1 predicted
Eigen::Matrix<double, 2 * (WAVE_COMPONENTS + 1), 2 * (WAVE_COMPONENTS + 1)>
    p_k_dash = Eigen::MatrixXd::Identity(2 * (WAVE_COMPONENTS + 1),
                                         2 * (WAVE_COMPONENTS + 1));
// P_k+1 actual
Eigen::Matrix<double, 2 * (WAVE_COMPONENTS + 1), 2 * (WAVE_COMPONENTS + 1)>
    p_k = Eigen::MatrixXd::Zero(2 * (WAVE_COMPONENTS + 1),
                                2 * (WAVE_COMPONENTS + 1));

Eigen::MatrixXd c_t(1, 2 * (WAVE_COMPONENTS + 1));  // C(t_0)
Eigen::Matrix2d a_i;                                // A_i(t_0)
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
ros::Publisher detected_output;

void PredictWaveFuture(double pred_time, double &time_of_msg, double &zeroth,
                       double &first, double &second) {
  double t_k = 0.0;
  double sine_output = 0.0;
  zeroth = 0.0;
  first = 0.0;
  second = 0.0;
  time_of_msg = pred_time;
  for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
    //------extracting frequency-----
    freq(0) = freq_components(i);

    ident_msg.frequency[i] = freq(0);

    //------extracting time----------
    // t_k = (ros::Time::now().toNSec() - msg_time[i]) * 1e-9;

    // --------------------------extracting phase------------------------
    phase(0) = atan2(2 * M_PI * freq(0) * x_t(2 * i), x_t((2 * i) + 1));
    // -------------------extracting amplitude-------------------------
    ident_msg.phase[i] = phase(0);

    sine_output = sin(phase(0));
    if (sine_output != 0.0) {
      amplitude(0) = x_t(2 * i) / sine_output;
      if (amplitude(0) > 100.0) {
        // "when the phase is very close to multiples of PI, the value of
        // sin(phase(0)) is very close to zero, about 1e-16 sometimes, and we
        // cannot set the limits based on phase because then real detections
        // with low phase get sent to the shadown realm of zero amplitude if
        // you filter it in the previous if statement. So, this is a
        // gatekeeping method that should prevent false positives. And that
        // method breaks the algorithm in some way that I did not investigate.
        // 0/10 would not recommend."
        //-One Parakh in Jan 2022 during quarantine
        amplitude(0) = 0.0;
      }
    } else {
      amplitude(0) = 0.0;
    }
    ident_msg.amplitude[i] = amplitude(0);
    t_k = ((ros::Time::now().toNSec() - msg_time) * 1e-9) + pred_time;
    // --------------- zeroth ------------------
    zeroth += amplitude(0) * sin((2 * M_PI * freq(0) * t_k) + phase(0));
    // --------------- first --------------------
    first += amplitude(0) * 2 * M_PI * freq(0) *
             cos((2 * M_PI * freq(0) * t_k) + phase(0));
    // --------------- second --------------------
    second += -amplitude(0) * pow(2 * M_PI * freq(0), 2) *
              sin((2 * M_PI * freq(0) * t_k) + phase(0));
  }
  // ------------------- adding random noise component -----------------
  zeroth += x_t(2 * WAVE_COMPONENTS);
}

bool PredictionServiceCallback(usv_estiplan::PredictionOutput::Request &req,
                               usv_estiplan::PredictionOutput::Response &res) {
  // ------------ predictions ----------------
  PredictWaveFuture(req.pred_time, req.pred_time, res.zeroth, res.first,
                    res.second);
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
  decoded_msg.header.stamp = ros::Time::now();
  decoded_msg.data = w_k(0);
  wave_decoded.publish(decoded_msg);
  if (_online_tune_) {
    ros::NodeHandle estiplan("~");
    if (!estiplan.getParam("q_2n",
                           q(2 * WAVE_COMPONENTS, 2 * WAVE_COMPONENTS))) {
      ROS_WARN("WAVE_PRED:KF_q_2n loading failed, quitting");
    }
    if (!estiplan.getParam("q_2n_1", q((2 * WAVE_COMPONENTS) + 1,
                                       (2 * WAVE_COMPONENTS) + 1))) {
      ROS_WARN("WAVE_PRED:KF_q_2n_1 loading failed, quitting");
    }
    if (!estiplan.getParam("r", r(0))) {
      ROS_WARN("WAVE_PRED:KF_r loading failed, quitting");
    }
    if (!estiplan.getParam("x_t_2n", x_t(2 * WAVE_COMPONENTS))) {
      ROS_WARN("WAVE_PRED:KF_x_t_2n loading failed, quitting");
    }
    if (!estiplan.getParam("x_t_2n_1", x_t(2 * WAVE_COMPONENTS + 1))) {
      ROS_WARN("WAVE_PRED:KF_x_t_2n_1 loading failed, quitting");
    }
    for (size_t i = 0; i < 2 * WAVE_COMPONENTS; i++) {
      if (!estiplan.getParam("q", q(i, i))) {
        ROS_WARN("WAVE_PRED:KF_q loading failed, quitting");
      }
    }
  }

  if (!first_start) {
    q_dash = 0.5 * ((phi * q * phi.transpose()) + q) * sample_time;
    p_k_dash = ((phi * p_k_dash * phi.transpose()) + q_dash).eval();
    l_k = p_k_dash * c_t.transpose() *
          (c_t * p_k_dash * c_t.transpose() + r).inverse();
    x_predicted = phi * x_t;
    w_k_hat = c_t * x_predicted;
    x_predicted = (x_predicted + (l_k * (w_k - w_k_hat))).eval();
    w_k_hat = c_t * x_predicted;
    p_k = (Eigen::MatrixXd::Identity(2 * (WAVE_COMPONENTS + 1),
                                     2 * (WAVE_COMPONENTS + 1)) -
           (l_k * c_t)) *
          p_k_dash;
    x_t = x_predicted;
    p_k_dash = p_k;
    msg_time = ros::Time::now().toNSec();
    wave_msg.data = w_k_hat(0);
    wave_msg.header.stamp = ros::Time::now();
    wave_observer.publish(wave_msg);
    PredictWaveFuture(t_future, wave_future_msg.pred_time,
                      wave_future_msg.zeroth, wave_future_msg.first,
                      wave_future_msg.second);
    wave_future_msg.header.stamp = ros::Time::now();
    wave_future.publish(wave_future_msg);
    if (_diagnostics_) {
      detected_output.publish(ident_msg);
    }
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
        parity_matrix[0][j] = true;  // row-0 for last_msg
        parity_matrix[1][i] = true;  // row-1 for new_msg
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
      x_t.block(2 * i, 0, 2, 1) = x_i;
      for (size_t k = 0; k < 2 * (WAVE_COMPONENTS + 1); k++) {
        p_k_dash(2 * i, k) = 0.0;
        p_k_dash(k, 2 * i) = 0.0;
        p_k_dash((2 * i) + 1, k) = 0.0;
        p_k_dash(k, (2 * i) + 1) = 0.0;
      }
      p_k_dash(2 * i, 2 * i) = 1.0;
      p_k_dash((2 * i) + 1, (2 * i) + 1) = 1.0;
      spot_avlb += 1;
    }
  }
  // updating Phi
  phi = (a_t * sample_time).exp();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wave_prediction", ros::init_options::AnonymousName);
  ros::NodeHandle estiplan("~");
  if (argc != 2) {
    ROS_ERROR("Specify the DOF arg in launch file");
    return -1;
  }
  dof_name = argv[1];
  if (!estiplan.getParam("sampling_freq", sampling_freq)) {
    ROS_ERROR("sampling_freq loading failed");
    return -1;
  }
  sample_time = 1 / sampling_freq;
  if (!estiplan.getParam("t_future", t_future)) {
    ROS_WARN("WAVE_PRED:t_future loading failed, using 0.0 default");
  }
  if (!estiplan.getParam("r", r(0))) {
    ROS_WARN("WAVE_PRED:KF_r loading failed, quitting");
    return -1;
  }
  if (!estiplan.getParam("x_t_2n", x_t(2 * WAVE_COMPONENTS))) {
    ROS_WARN("WAVE_PRED:KF_x_t_2n loading failed, quitting");
    return -1;
  }
  if (!estiplan.getParam("x_t_2n_1", x_t(2 * WAVE_COMPONENTS + 1))) {
    ROS_WARN("WAVE_PRED:KF_x_t_2n_1 loading failed, quitting");
    return -1;
  }
  if (!estiplan.getParam("p_k_2n",
                         p_k_dash(2 * WAVE_COMPONENTS, 2 * WAVE_COMPONENTS))) {
    ROS_WARN("WAVE_PRED:KF_p_k_2n loading failed, quitting");
    return -1;
  }
  if (!estiplan.getParam("p_k_2n_1", p_k_dash(2 * WAVE_COMPONENTS + 1,
                                              2 * WAVE_COMPONENTS + 1))) {
    ROS_WARN("WAVE_PRED:KF_p_k_2n_1 loading failed, quitting");
    return -1;
  }
  if (!estiplan.getParam("q_2n", q(2 * WAVE_COMPONENTS, 2 * WAVE_COMPONENTS))) {
    ROS_WARN("WAVE_PRED:KF_q_2n loading failed, quitting");
    return -1;
  }
  if (!estiplan.getParam(
          "q_2n_1", q((2 * WAVE_COMPONENTS) + 1, (2 * WAVE_COMPONENTS) + 1))) {
    ROS_WARN("WAVE_PRED:KF_q_2n_1 loading failed, quitting");
    return -1;
  }
  if (!estiplan.getParam("online_tune", _online_tune_)) {
    ROS_WARN("WAVE_PRED:online_tune loading failed, default is false");
  }
  if (!estiplan.getParam("diagnostics", _diagnostics_)) {
    ROS_WARN("WAVE_PRED:diagnostics loading failed, default is false");
  }
  a_i(0) = 0.0;
  a_i(2) = 1.0;
  a_i(3) = 0.0;
  for (size_t i = 0; i < 2 * WAVE_COMPONENTS; i++) {
    if (!estiplan.getParam("q", q(i, i))) {
      ROS_WARN("WAVE_PRED:KF_q loading failed, quitting");
      return -1;
    }
  }

  for (size_t i = 0; i < WAVE_COMPONENTS + 1; i++) {
    c_t(2 * i) = 1.0;
    c_t((2 * i) + 1) = 0.0;
  }

  ros::Subscriber sub = estiplan.subscribe("fft_in", 1000, FftCallback);
  ros::Subscriber sub_pose =
      estiplan.subscribe("odometry_in", 1000, OdomCallback);
  wave_observer = estiplan.advertise<usv_estiplan::Float64Stamped>(
      dof_name + "_observer", 1000);
  wave_decoded = estiplan.advertise<usv_estiplan::Float64Stamped>(
      dof_name + "_decoded", 1000);
  wave_future =
      estiplan.advertise<usv_estiplan::Wavefuture>(dof_name + "_future", 1000);
  detected_output = estiplan.advertise<usv_estiplan::Fftarray>(
      dof_name + "_idenitification", 1000);
  // ros::ServiceServer prediction_srv = estiplan.advertiseService(
  //     "/wave_prediction/" + dof_name, PredictionServiceCallback);
  ros::Publisher prediction_publisher =
      estiplan.advertise<usv_estiplan::Wavefuture>(dof_name + "_predictions",
                                                   1000);
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    if (!horizon_loaded) {
      if (!estiplan.getParam("/lmpc_pred_horizon", pred_horizon)) {
        ROS_WARN("WAVE_PRED : pred_horizon loading failed");
      } else {
        horizon_loaded = true;
      }
    }
    double pred_time = 0.01;
    for (size_t i = 0; i < pred_horizon; i++) {
      PredictWaveFuture(pred_time, wave_future_msg.pred_time,
                        wave_future_msg.zeroth, wave_future_msg.first,
                        wave_future_msg.second);
      prediction_publisher.publish(wave_future_msg);
      pred_time += 0.01;
    }

    if (ros::isShuttingDown()) {
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
