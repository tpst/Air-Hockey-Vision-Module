#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "tracking.h"

using namespace cv;
using namespace std;

kalman_filter::kalman_filter(void) 
	: F(4, 4), H(2, 4), P(4, 4), R(2, 2), x(4, 1), I(4, 4), Q(4, 4)
{
	// initial state (location, velocity)
	x << 0.0, 0.0, 0.0, 0.0;

	// Transition matrix (x, y, vx, vy)
	F << 1.0, 0.0, 0.04, 0.0,
		 0.0, 1.0, 0.0, 0.04, 
		 0.0, 0.0, 1.0, 0.0,
		 0.0, 0.0, 0.0, 1.0;

	// Measurement matrix (x, y)
	H << 1.0, 0.0, 0.0, 0.0,
		 0.0, 1.0, 0.0, 0.0;

	// Identity matrix
	I << 1.0, 0.0, 0.0, 0.0,
		 0.0, 1.0, 0.0, 0.0,
		 0.0, 0.0, 1.0, 0.0,
		 0.0, 0.0, 0.0, 1.0;

	// Covariance matrix (initial uncertainty)
	P << 1, 0.0, 0.0, 0.0, 
		 0.0, 1, 0.0, 0.0,
		 0.0, 0.0, 1, 0.0, 
		 0.0, 0.0, 0.0, 1;
	P = P*1e-1;

	Q << 1, 0.0, 0.0, 0.0, 
		 0.0, 1, 0.0, 0.0,
		 0.0, 0.0, 1, 0.0, 
		 0.0, 0.0, 0.0, 1;
	Q = P*1e-5;

	// Measurement noise covariance (x, y, vx, vy)
	R << 1, 0.0,
		 0.0, 1;
	R = R*10e-9;
}

Mat kalman_filter::filter(cv::Point2d pos)
{
	cv::Mat_<float> Z(2, 1); // measurement matrix [x, y]
	Z << pos.x, pos.y;

	cout << "Measurement = " << endl << " " << Z << endl << endl;

	// Time Update (Prediction)
	// ========================
	// project the state ahead
	x = (F * x);
	cout << " 1. State = " << endl << " " << x << endl << endl;

	// project the error covariance ahead
	P = F*P*F.t() + Q;
	//cout << " 1. Next covariance = " << endl << " " << P << endl << endl;

	// Measurement Update (Correction)
	// ==============================
	// Compute the kalman gain

	S = H * P * H.t() + R;
	K = (P*H.t())*S.inv();
	//cout << " 2. Kalman Gain: " << endl << " " << K << endl << endl;

	// Update the estimate via Z
	x = x + (K * (Z - (H * x)));
	cout << "2. update estimate x " << endl << " " << x << endl << endl;

	// Update Error covariance
	P = (I - K*H)*P;
	//cout << "2. update error covariance (P)" << endl << " " << P << endl << endl;
	return x;
}