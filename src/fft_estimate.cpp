#include <fftw3.h>
#include <iostream>

using namespace std;

#define REAL 0
#define IMAG 1

int main(){
	int n = 5;
	fftw_complex in[n];
	fftw_complex out[n];
	for(int i=0;i<n;i++){
		in[i][REAL] = i + 1;
		out[i][IMAG] = 0;
	}
	fftw_plan plan = fftw_plan_dft_1d(n,in,out,FFTW_FORWARD, FFTW_ESTIMATE);
	fftw_execute(plan);
	fftw_destroy_plan(plan);
	fftw_cleanup();
	cout << "FFT = " << endl;
	for (int i=0;i<n;i++){
			cout << out[i][REAL] << "-" << abs(out[i][IMAG]) << endl;
	}

	return 0;
}