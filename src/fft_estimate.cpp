#include <fftw3.h>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include <cmath>
#include "geometry_msgs/Vector3.h"
#include "usv_estiplan/Fftoutput.h"
#include <fstream>
#include <queue>

using namespace std;

#define REAL 0
#define IMAG 1

const int max_queue = 10000;
const int samp_freq = 83;
const int wave_components = 10;
queue<float> x_queue;
queue<float> y_queue;
uint64_t iter_global = 0;
// fftw_complex in_x[N];
// fftw_complex in_y[N];
sensor_msgs::Imu imu_data{};
geometry_msgs::Vector3 fft_data{};
usv_estiplan::Fftoutput fft_msg{};

void queue_things(queue<float> &queue_in,float push_in){
	while(queue_in.size() > max_queue){
		queue_in.pop();
	}
		queue_in.push(push_in);
}

void imuCallback(sensor_msgs::Imu imu_data)
{
	queue_things(x_queue,imu_data.angular_velocity.x);
	queue_things(y_queue,imu_data.angular_velocity.y);
}
void convert_que_to_array(queue<float> dupl,fftw_complex *fftw_in){
	int fft_size = dupl.size(); // using dupl.size() multiple times was causing for loop to use garbage i values.
	for(int i=0;i < fft_size;i++){
		fftw_in[i][REAL] = dupl.front();
		fftw_in[i][IMAG] = 0.0;
		dupl.pop();
	}

}


int main(int argc, char **argv){

	ros::init(argc, argv, "fft_estimate");
	ros::NodeHandle estiplan;
	ros::Subscriber sub = estiplan.subscribe("/wamv/imu", 1000, imuCallback);
	ros::Publisher fft_transform = estiplan.advertise<usv_estiplan::Fftoutput>("/fft_output/", 1000);
	// ros::Publisher fft_transform = estiplan.advertise<geometry_msgs::Vector3>("/fft_output/y", 1000);
	ros::Rate loop_rate(0.1);
	ofstream fft_output_capt;
	fft_output_capt.open("fft_data.csv");
	x_queue.push(0); // to prevent core dump
	while(ros::ok()){
		// one fft block begin
		int fft_size = x_queue.size();
		fftw_complex in_fftw_x[fft_size] = {};
		fftw_complex out_fftw_x[fft_size] = {};
		convert_que_to_array(x_queue, in_fftw_x);
		fftw_plan plan = fftw_plan_dft_1d(fft_size,in_fftw_x,out_fftw_x,FFTW_FORWARD, FFTW_ESTIMATE);
		fftw_execute(plan);
		fftw_destroy_plan(plan);
		fftw_cleanup();
		// one fft block end
		// one fft block begin
		// fft_size = y_queue.size();
		// fftw_complex in_fftw_y[fft_size] = {};
		// fftw_complex out_fftw_y[fft_size] = {};
		// convert_que_to_array(y_queue, in_fftw_y);
		// plan = fftw_plan_dft_1d(fft_size,in_fftw_y,out_fftw_y,FFTW_FORWARD, FFTW_ESTIMATE);
		// fftw_execute(plan);
		// fftw_destroy_plan(plan);
		// fftw_cleanup();
		// one fft block end

		double x_output[wave_components][3];
		double y_output[wave_components][3];
		
		cout << "FFT accuracy(Hz) : " << float(samp_freq/2.0f) / (fft_size/2.0f) << endl;
		int filler = 0;
		for (int i=0;i<fft_size/2;i++){
			iter_global += 1;
			fft_data.x = i * float(samp_freq/2) / (fft_size/2);
			fft_data.y = 2.0f*sqrt(pow(out_fftw_x[i][REAL],2) + pow(out_fftw_x[i][IMAG],2))/float(fft_size);
			fft_data.z = atan2(out_fftw_x[i][IMAG],out_fftw_x[i][REAL]);
			if(fft_data.y > 0.02 && filler < 10){
				fft_msg.frequency[filler] = fft_data.x;
				fft_msg.amplitude[filler] = fft_data.y;
				fft_msg.phase[filler] = fft_data.z;
				filler += 1;
			// fft_output_capt << iter_global << ";" << fft_data.x << ";" << fft_data.y << ";" << fft_data.z <<'\n';
			}
			// ros::Duration(0.01).sleep();
		}
		fft_transform.publish(fft_msg);
		if(ros::isShuttingDown()){
			fft_output_capt.close();
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}