#include <fftw3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <queue>
#include <sstream>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "usv_estiplan/Fftarray.h"
#include "usv_estiplan/Fftresult.h"
#include "usv_estiplan/Float64Stamped.h"

using namespace std;

#define REAL 0
#define IMAG 1

float freq_accuracy = 0.01;
int MAX_QUEUE = 10000;
int samp_freq = 100;
float fft_interval = 0.2;
const int WAVE_COMPONENTS = 20;
float fft_threshold = 0.02;
bool _hanning_window_ = false;
bool _debug_ = false;
int _peak_lag_ = 0;
float _peak_threshold_ = 0.0;
float _peak_influence_ = 0.0;
queue<float> x_queue;
queue<float> y_queue;
queue<float> z_queue;
queue<float> roll_queue;
queue<float> pitch_queue;
queue<float> yaw_queue;
geometry_msgs::PoseStamped pose_data{};
usv_estiplan::Fftresult fft_result{};
usv_estiplan::Fftarray fft_array{};
usv_estiplan::Float64Stamped accuracy_msg;
usv_estiplan::Float64Stamped accuracy_target_msg;
vector<vector<double>> wave_vec;
std::ofstream debug_file_x;
std::ofstream debug_file_y;
std::ofstream debug_file_z;
std::ofstream debug_file_roll;
std::ofstream debug_file_pitch;
std::ofstream debug_file_yaw;

usv_estiplan::Fftarray processFft(int fft_size, fftw_complex *out_fftw,
                                  std::ofstream &debug_file);
void executeFft(queue<float> queue_in, fftw_complex *out_fftw);
void detectPeaks();
double findMean(std::vector<double> input);
double findStdDev(std::vector<double> input);

double findMean(std::vector<double> input) {
  double sum_of_elems = 0.0;
  sum_of_elems = std::accumulate(input.begin(), input.end(), 0.0);
  return sum_of_elems / input.size();
}

double findStdDev(std::vector<double> input) {
  double mean = findMean(input);
  double std_dev = 0.0;
  for (size_t i = 0; i < input.size(); i++) {
    std_dev += (mean - input[i]) * (mean - input[i]);
  }
  return sqrt(std_dev / (input.size() - 1));
}

std::vector<int> detectPeaks(std::vector<double> input) {
  // lag 5 for the smoothing functions
  int lag = _peak_lag_;
  // 3.5 standard deviations for signal
  float threshold = _peak_threshold_;
  // between 0 and 1, where 1 is normal influence, 0.5 is half
  float influence = _peak_influence_;

  if (input.size() <= lag + 2) {
    std::vector<int> emptyVec;
    return emptyVec;
  }

  // Initialise variables
  std::vector<int> signals(input.size(), 0.0);
  std::vector<double> filteredY(input.size(), 0.0);
  std::vector<double> avgFilter(input.size(), 0.0);
  std::vector<double> stdFilter(input.size(), 0.0);
  std::vector<double> subVecStart(input.begin(), input.begin() + lag);
  avgFilter[lag] = findMean(subVecStart);
  stdFilter[lag] = findStdDev(subVecStart);

  for (size_t i = lag + 1; i < input.size(); i++) {
    if (std::abs(input[i] - avgFilter[i - 1]) > threshold * stdFilter[i - 1]) {
      if (input[i] > avgFilter[i - 1]) {
        signals[i] = 1;  //# Positive signal
      } else {
        signals[i] = -1;  //# Negative signal
      }
      // Make influence lower
      filteredY[i] = influence * input[i] + (1 - influence) * filteredY[i - 1];
    } else {
      signals[i] = 0;  //# No signal
      filteredY[i] = input[i];
    }
    // Adjust the filters
    std::vector<double> subVec(filteredY.begin() + i - lag,
                               filteredY.begin() + i);
    avgFilter[i] = findMean(subVec);
    stdFilter[i] = findStdDev(subVec);
  }
  return signals;
}

void queue_things(queue<float> &queue_in, float push_in) {
  while (queue_in.size() > MAX_QUEUE - 1) {
    queue_in.pop();
  }
  queue_in.push(push_in);
}

void odomCallback(const geometry_msgs::PoseStamped &msg) {
  queue_things(x_queue, msg.pose.position.x);
  queue_things(y_queue, msg.pose.position.y);
  queue_things(z_queue, msg.pose.position.z);
  tf2::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  queue_things(roll_queue, roll);
  queue_things(pitch_queue, pitch);
  queue_things(yaw_queue, yaw);
}
void convert_que_to_array(queue<float> dupl, fftw_complex *fftw_in) {
  int fft_size = dupl.size();  // using dupl.size() multiple times was causing
                               // for loop to use garbage i values.
  for (int i = 0; i < fft_size; i++) {
    if (_hanning_window_) {
      fftw_in[i][REAL] =
          pow(sin(M_PI * i / fft_size), 2) * dupl.front();  // hanning window
    } else {
      fftw_in[i][REAL] = dupl.front();
    }
    fftw_in[i][IMAG] = 0.0;
    dupl.pop();
  }
}

bool sortcol(const vector<double> &v1, const vector<double> &v2) {
  return v1[1] > v2[1];
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fft_estimate");
  ros::NodeHandle estiplan("~");
  if (!estiplan.getParam("sampling_freq", samp_freq)) {
    ROS_ERROR("sampling_freq loading failed");
    return -1;
  }
  if (!estiplan.getParam("fft_interval", fft_interval)) {
    ROS_ERROR("fft_interval loading failed");
    return -1;
  }
  if (!estiplan.getParam("fft_threshold", fft_threshold)) {
    ROS_ERROR("fft_threshold loading failed, using 0.02 default");
  }
  if (!estiplan.getParam("freq_accuracy", freq_accuracy)) {
    ROS_ERROR("freq_accuracy loading failed, using 0.01 default");
  }
  if (!estiplan.getParam("hanning_window", _hanning_window_)) {
    ROS_ERROR("FFT: _hanning_window_ loading failed");
  }
  if (!estiplan.getParam("debug", _debug_)) {
    ROS_WARN("FFT: _debug_ loading failed, defaulting to false");
  }

  if (!estiplan.getParam("peak_lag", _peak_lag_)) {
    ROS_WARN("FFT: _peak_lag_ loading failed, defaulting to false");
  }
  if (!estiplan.getParam("peak_influence", _peak_influence_)) {
    ROS_WARN("FFT: _peak_influence_ loading failed, defaulting to false");
  }
  if (!estiplan.getParam("peak_threshold", _peak_threshold_)) {
    ROS_WARN("FFT: _peak_threshold_ loading failed, defaulting to false");
  }
  if (_debug_) {
    debug_file_x.open("/tmp/debug_fft_x.csv");
    debug_file_y.open("/tmp/debug_fft_y.csv");
    debug_file_z.open("/tmp/debug_fft_z.csv");
    debug_file_roll.open("/tmp/debug_fft_roll.csv");
    debug_file_pitch.open("/tmp/debug_fft_pitch.csv");
    debug_file_yaw.open("/tmp/debug_fft_yaw.csv");
  }
  std::vector<double> test_vector = {1,   1.5, 1,   1.5, 3,  2,
                                     2.0, 1.3, 1.4, 1.3, 1.2};
  std::cout << "Test Mean is: " << findMean(test_vector)
            << " Test Std Dev: " << findStdDev(test_vector) << "\n";
  std::string header_string = "frequency;amplitude;phase;signal\n";
  debug_file_pitch << header_string;
  debug_file_roll << header_string;
  debug_file_yaw << header_string;
  debug_file_z << header_string;
  debug_file_y << header_string;
  debug_file_x << header_string;
  MAX_QUEUE = samp_freq / freq_accuracy;
  ros::Subscriber sub = estiplan.subscribe("odom_in", 1000, odomCallback);
  ros::Publisher fft_transform =
      estiplan.advertise<usv_estiplan::Fftresult>("fft_out", 1000);
  ros::Publisher fft_accuracy_pub =
      estiplan.advertise<usv_estiplan::Float64Stamped>("fft_accuracy", 1000);
  ros::Publisher fft_accuracy_target_pub =
      estiplan.advertise<usv_estiplan::Float64Stamped>("fft_accuracy_target",
                                                       1000);
  ros::Rate loop_rate(1 / fft_interval);
  // ofstream fft_output_capt;
  // fft_output_capt.open("fft_data.csv");
  queue_things(x_queue, 0.0);      // to prevent core dump
  queue_things(y_queue, 0.0);      // to prevent core dump
  queue_things(z_queue, 0.0);      // to prevent core dump
  queue_things(roll_queue, 0.0);   // to prevent core dump
  queue_things(pitch_queue, 0.0);  // to prevent core dump
  queue_things(yaw_queue, 0.0);    // to prevent core dump
  for (size_t i = 0; i < MAX_QUEUE / 2; i++) {
    wave_vec.push_back({0, 0, 0});  // Freq, Amp, Phase
  }
  while (ros::ok()) {
    int fft_size = 0;
    //----------- x fft ---------------//
    fft_size = x_queue.size();
    fftw_complex out_fftw_x[fft_size] = {};
    executeFft(x_queue, out_fftw_x);
    //----------- y fft ---------------//
    fftw_complex out_fftw_y[fft_size] = {};
    executeFft(y_queue, out_fftw_y);
    //----------- z fft ---------------//
    fftw_complex out_fftw_z[fft_size] = {};
    executeFft(z_queue, out_fftw_z);
    //----------- roll fft ---------------//
    fftw_complex out_fftw_roll[fft_size] = {};
    executeFft(roll_queue, out_fftw_roll);
    //----------- pitch fft ---------------//
    fftw_complex out_fftw_pitch[fft_size] = {};
    executeFft(pitch_queue, out_fftw_pitch);
    //----------- yaw fft ---------------//
    fftw_complex out_fftw_yaw[fft_size] = {};
    executeFft(yaw_queue, out_fftw_yaw);

    accuracy_msg.data = float(samp_freq / 2.0f) / (fft_size / 2.0f);
    accuracy_msg.header.stamp = ros::Time::now();
    accuracy_target_msg.header.stamp = ros::Time::now();
    accuracy_target_msg.data = freq_accuracy;
    fft_accuracy_pub.publish(accuracy_msg);
    fft_accuracy_target_pub.publish(accuracy_target_msg);
    cout << "FFT accuracy(Hz) : " << accuracy_msg.data << endl;
    fft_result.header.stamp = ros::Time::now();
    fft_result.x = processFft(fft_size, out_fftw_x, debug_file_x);
    fft_result.y = processFft(fft_size, out_fftw_y, debug_file_y);
    fft_result.z = processFft(fft_size, out_fftw_z, debug_file_z);
    fft_result.roll = processFft(fft_size, out_fftw_roll, debug_file_roll);
    fft_result.pitch = processFft(fft_size, out_fftw_pitch, debug_file_pitch);
    fft_result.yaw = processFft(fft_size, out_fftw_yaw, debug_file_yaw);
    // debug_file.close();

    fft_transform.publish(fft_result);
    if (ros::isShuttingDown()) {
      std::cout << "Closing files" << std::endl;
      debug_file_x.close();
      debug_file_y.close();
      debug_file_z.close();
      debug_file_pitch.close();
      debug_file_roll.close();
      debug_file_yaw.close();

      // fft_output_capt.close();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void executeFft(queue<float> queue_in, fftw_complex *out_fftw) {
  int fft_size = queue_in.size();
  fftw_complex in_fftw[fft_size] = {};
  convert_que_to_array(queue_in, in_fftw);
  fftw_plan plan = fftw_plan_dft_1d(fft_size, in_fftw, out_fftw, FFTW_FORWARD,
                                    FFTW_ESTIMATE);
  fftw_execute(plan);
  fftw_destroy_plan(plan);
  fftw_cleanup();
}

usv_estiplan::Fftarray processFft(int fft_size, fftw_complex *out_fftw,
                                  std::ofstream &debug_file) {
  for (size_t i = 0; i < MAX_QUEUE / 2; i++) {
    wave_vec[i][0] = 0.0;
    wave_vec[i][1] = 0.0;
    wave_vec[i][2] = 0.0;
  }
  std::string temp_string = "";
  for (int i = 1; i < fft_size / 2; i++) {
    // Frequency
    wave_vec[i][0] = i * float(samp_freq / 2) / (fft_size / 2);
    // Amplitude
    wave_vec[i][1] =
        2.0f * sqrt(pow(out_fftw[i][REAL], 2) + pow(out_fftw[i][IMAG], 2)) /
        float(fft_size);
    // Phase
    wave_vec[i][2] = atan2(out_fftw[i][IMAG], out_fftw[i][REAL]);
    // DC component is doubled up so we reduce it again
    // Its phase is set to PI/2 to prevent it from disappearing
    if (i == 0) {
      wave_vec[i][1] = wave_vec[i][1] / 2;
      wave_vec[i][2] = M_PI / 2;
    }
  }
  std::vector<double> flat_vec;
  for (size_t i = 1; i < fft_size / 2; i++) {
    flat_vec.push_back(wave_vec[int((fft_size / 2)) - i][1]);
  }

  std::vector<int> peak_signal = detectPeaks(flat_vec);
  for (int i = 1; i < fft_size / 2; i++) {
    if (_debug_) {
      temp_string += std::to_string(wave_vec[i][0]) + ";";
      temp_string += std::to_string(wave_vec[i][1]) + ";";
      temp_string += std::to_string(wave_vec[i][2]) + ";";
      temp_string +=
          std::to_string(wave_vec[i][1] * peak_signal[(fft_size / 2) - i - 1]) +
          "\n";
    }
  }

  sort(wave_vec.begin(), wave_vec.end(), sortcol);
  usv_estiplan::Fftarray fft_array;
  //------------Fix for Issue #8-----------
  double fft_threshold_ref = 0.0;
  if (wave_vec[0][0] == 0) {
    fft_threshold_ref = fft_threshold * wave_vec[1][1];
  } else {
    fft_threshold_ref = fft_threshold * wave_vec[0][1];
  }

  for (size_t i = 0; i < WAVE_COMPONENTS; i++) {
    if (wave_vec[i][1] > fft_threshold_ref) {
      fft_array.frequency[i] = wave_vec[i][0];
      fft_array.amplitude[i] = wave_vec[i][1];
      fft_array.phase[i] = wave_vec[i][2];
      // temp_string += std::to_string(fft_array.frequency[i]) + ";";
      // temp_string += std::to_string(fft_array.amplitude[i]) + ";";
    } else {
      fft_array.frequency[i] = 0.0;
      fft_array.amplitude[i] = 0.0;
      fft_array.phase[i] = 0.0;
    }
  }
  if (_debug_) {
    // temp_string += "\n";
    debug_file << temp_string;
  }
  return fft_array;
}