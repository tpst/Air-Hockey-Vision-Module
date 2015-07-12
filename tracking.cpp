#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "tracking.h"

using namespace cv;
using namespace std;

puck_filter::puck_filter(Point2d pos) 
	: F(4, 4), H(2, 4), P(4, 4), R(2, 2), x(4, 1), I(4, 4), Q(4, 4)
 {
 // initial state (location, velocity)
	x << pos.x, pos.y, 0.0, 0.0;

	// Transition matrix (x, y, vx, vy)
	F << 1.0, 0.0, 4.0, 0.0, //better performance using 4.0 than 0.4 ?
		 0.0, 1.0, 0.0, 4.0, 
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
	P = P*1e-2; //1e-2

	Q << 1/4, 0.0, 1/2, 0.0, 
		 0.0, 1/4, 0.0, 1/2,
		 0.0, 0.0, 1/4, 0.0, 
		 0.0, 0.0, 0.0, 1/4;
	Q = P*1e-3; //1e-4

	// Measurement noise covariance (x, y, vx, vy)
	R << 1e-6, 0.0,
		 0.0, 1e-6; // 1e07
 }

// Kalman filter update method
Mat puck_filter::filter(cv::Point2d pos)
{
	cv::Mat_<float> z(2, 1); // measurement matrix [x, y]
	z << pos.x, pos.y;

	//cout << "Measurement = " << endl << " " << z << endl << endl;

	// Time Update (Prediction)
	// ========================
	// project the state ahead
	x = (F * x);
	//cout << " 1. State = " << endl << " " << x << endl << endl;

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
	x = x + (K * (z - (H * x)));
	//cout << "2. update estimate x " << endl << " " << x << endl << endl;

	// Update Error covariance
	P = (I - K*H)*P;
	//cout << "2. update error covariance (P)" << endl << " " << P << endl << endl;

	return x; //-- return state estimation
}


// Predict when there is no measurement data available
Mat puck_filter::filter() {

	// Time Update (Prediction)
	// ========================
	// project the state ahead
	x = (F * x);
	//cout << " 1. State = " << endl << " " << x << endl << endl;

	// project the error covariance ahead
	P = F*P*F.t() + Q;
	//cout << " 1. Next covariance = " << endl << " " << P << endl << endl;

	return x; //-- return state estimation
}

prediction_filter::prediction_filter(Point2d pos) 
	: F(3, 3), H(3, 3), P(3, 3), R(3, 3), x(3, 1), I(3, 3), Q(3, 3)
 {
 // initial state (location, velocity)
	x << pos.x, pos.y, 0.0;

	// Transition matrix (x, y, theta)
	F << 1.0, 0.0, 0.0,
		 0.0, 1.0, 0.0, 
		 0.0, 0.0, 1.0;

	// Measurement matrix (x, y)
	H << 1.0, 0.0, 0.0,
		 0.0, 1.0, 0.0,
		 0.0, 0.0, 1.0;

	// Identity matrix
	I << 1.0, 0.0, 0.0, 
		 0.0, 1.0, 0.0, 
		 0.0, 0.0, 1.0;

	// Covariance matrix (initial uncertainty)
	P << 1.0, 0.0, 0.0, 
		 0.0, 1, 0.0, 
		 0.0, 0.0, 1;
	P = P*1e-5; // 1e-4

	Q << 1/4, 0.0, 0.0,
		 0.0, 1/4, 0.0,
		 0.0, 0.0, 1/4; 
	Q = P*1e-6; // 1e-6

	// Measurement noise covariance (x, y, vx, vy)
	R << 1e-6, 0.0, 0.0,
		 0.0, 1e-6, 0.0, 
		 0.0, 0.0, 1e-6; //1e-4
 }

// Kalman filter update method
Mat prediction_filter::filter(cv::Point2d pos)
{
	cv::Mat_<float> z(3, 1); // measurement matrix [x, y, theta]
	z << pos.x, pos.y, 0.0;
	
	// prevent drifting errors 
	if(x[0][0] != pos.x) x[0][0] = pos.x;

	// Time Update (Prediction)
	// ========================
	// project the state ahead
	x = (F * x);
	
	// project the error covariance ahead
	P = F*P*F.t() + Q;

	// Measurement Update (Correction)
	// ==============================
	// Compute the kalman gain

	S = H * P * H.t() + R;
	K = (P*H.t())*S.inv();

	// Update the estimate via Z
	x = x + (K * (z - (H * x)));

	// Update Error covariance
	P = (I - K*H)*P;

	return x; //-- return state estimation
}



