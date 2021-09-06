// #include <fftw3.h>
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "usv_estiplan/Fftoutput.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <random>
#include <sstream>

using namespace std;

#define REAL 0
#define IMAG 1

// const int max_queue = 10000;
// const int samp_freq = 83;
const int wave_components = 10;
const float CONSTANT_OFFSET = 0.2;
// queue<float> x_queue;
// queue<float> y_queue;
uint64_t iter_global = 0;
// fftw_complex in_x[N];
// fftw_complex in_y[N];
sensor_msgs::Imu imu_msg{};
// geometry_msgs::Vector3 fft_data{};
// usv_estiplan::Fftoutput fft_msg{};
double msg_time;
// std_msgs::Float64 wave_msg;
double amplitude[wave_components] = {0.1,  0.2,  0.15, 0.7,  0.65,
                                     0.35, 0.23, 0.5,  0.05, 0.85};
double frequency[wave_components] = {0.1,  0.4,  0.25, 0.31, 0.05,
                                     0.35, 0.13, 0.04, 0.48, 0.6};
double phase[wave_components] = {0.1, 0.2, 0.3, 0.4, 0.5,
                                 0.6, 0.7, 0.8, 0.9, 1.0};
// void queue_things(queue<float> &queue_in,float push_in){
// 	while(queue_in.size() > max_queue){
// 		queue_in.pop();
// 	}
// 		queue_in.push(push_in);
// }

// void fftCallback(usv_estiplan::Fftoutput in_msg)
// {
// 	fft_msg = in_msg;
// 	msg_time = ros::Time::now().toNSec();
// }
// void convert_que_to_array(queue<float> dupl,fftw_complex *fftw_in){
// 	int fft_size = dupl.size(); // using dupl.size() multiple times was
// causing for loop to use garbage i values. 	for(int i=0;i < fft_size;i++){
// 		fftw_in[i][REAL] = dupl.front();
// 		fftw_in[i][IMAG] = 0.0;
// 		dupl.pop();
// 	}

// }

int main(int argc, char **argv) {

  ros::init(argc, argv, "wave_generator");
  ros::NodeHandle estiplan;
  // ros::Subscriber sub = estiplan.subscribe("/fft_output", 1000, fftCallback);
  ros::Publisher wave_generator =
      estiplan.advertise<sensor_msgs::Imu>("/wamv/imu", 1000);
  // ros::Publisher fft_transform =
  // estiplan.advertise<geometry_msgs::Vector3>("/fft_output/y", 1000);
  ros::Rate loop_rate(100.0);
  double wave_output;
  double time_elap = 0;
  while (ros::ok()) {
    wave_output = 0;
    for (size_t i = 0; i < wave_components; i++) {
      time_elap = ros::Time::now().toNSec() * 1e-9;
      wave_output +=
          amplitude[i] * sin((frequency[i] * 2 * M_PI * time_elap) + phase[i]);
    }
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(-0.3, 0.3);
    imu_msg.angular_velocity.x = wave_output + dist(mt) + CONSTANT_OFFSET;

    // ofstream fft_output_capt;
    // fft_output_capt.open("fft_data.csv");
    // x_queue.push(0); // to prevent core dump
    // 	// one fft block begin
    // 	int fft_size = x_queue.size();
    // 	fftw_complex in_fftw_x[fft_size] = {};
    // 	fftw_complex out_fftw_x[fft_size] = {};
    // 	convert_que_to_array(x_queue, in_fftw_x);
    // 	fftw_plan plan =
    // fftw_plan_dft_1d(fft_size,in_fftw_x,out_fftw_x,FFTW_FORWARD,
    // FFTW_ESTIMATE); 	fftw_execute(plan); 	fftw_destroy_plan(plan);
    // 	fftw_cleanup();
    // one fft block end
    // one fft block begin
    // fft_size = y_queue.size();
    // fftw_complex in_fftw_y[fft_size] = {};
    // fftw_complex out_fftw_y[fft_size] = {};
    // convert_que_to_array(y_queue, in_fftw_y);
    // plan = fftw_plan_dft_1d(fft_size,in_fftw_y,out_fftw_y,FFTW_FORWARD,
    // FFTW_ESTIMATE); fftw_execute(plan); fftw_destroy_plan(plan);
    // fftw_cleanup();
    // one fft block end

    // double x_output[wave_components][3];
    // double y_output[wave_components][3];

    // cout << "FFT accuracy(Hz) : " << float(samp_freq/2.0f) / (fft_size/2.0f)
    // << endl; int filler = 0; for (int i=0;i<fft_size/2;i++){
    // iter_global
    // += 1; 	fft_data.x = i * float(samp_freq/2) / (fft_size/2);
    // fft_data.y = 2.0f*sqrt(pow(out_fftw_x[i][REAL],2) +
    // pow(out_fftw_x[i][IMAG],2))/float(fft_size); 	fft_data.z =
    // atan2(out_fftw_x[i][IMAG],out_fftw_x[i][REAL]); 	if(fft_data.y > 0.02 &&
    // filler < 10){ 		fft_msg.frequency[filler] = fft_data.x;
    // 		fft_msg.amplitude[filler] = fft_data.y;
    // 		fft_msg.phase[filler] = fft_data.z;
    // 		filler += 1;
    // 	// fft_output_capt << iter_global << ";" << fft_data.x << ";" <<
    // fft_data.y << ";" << fft_data.z <<'\n';
    // 	}
    // ros::Duration(0.01).sleep();

    wave_generator.publish(imu_msg);
    if (ros::isShuttingDown()) {
      // fft_output_capt.close();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}