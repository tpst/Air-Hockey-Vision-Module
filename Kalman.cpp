//#include "Kalman.h"
//#include "visModule.h"
//
//kalman_filter::kalman_filter(void)
//	: F(4, 4), H(2, 4), P(4, 4), R(2, 2), x(4,1), I(4, 4), K_c(4, 2)
//{
//	// initial state (location, velocity)
//	x << 0.0, 0.0, 0.0, 0.0;
//
//	// Transition matrix (x, y, vx, vy)
//	F << 1.0, 0.0, 0.1, 0.0,
//		 0.0, 1.0, 0.0, 0.1, 
//		 0.0, 0.0, 1.0, 0.0,
//		 0.0, 0.0, 0.0, 1.0;
//
//	// Measurement matrix (x, y)
//	H << 1.0, 0.0, 0.0, 0.0,
//		 0.0, 1.0, 0.0, 0.0;
//
//	// Identity matrix
//	I << 1.0, 0.0, 0.0, 0.0,
//		 0.0, 1.0, 0.0, 0.0,
//		 0.0, 0.0, 1.0, 0.0,
//		 0.0, 0.0, 0.0, 1.0;
//
//	// initial uncertainty
//	P << 1000, 0.0, 0.0, 0.0, 
//		 0.0, 1000, 0.0, 0.0,
//		 0.0, 0.0, 1000, 0.0, 
//		 0.0, 0.0, 0.0, 1000;
//
//	// Process noise covariance (x, y, vx, vy)
//	R << 1e-1, 0.0,
//		 0.0, 1e-1;
//
//	K_c << 0.0707, 0,
//		   0, 0.0707,
//		   0.019, 0,
//		   0, 0.019;
//	
//}
//
//Mat kalman_filter::filter(cv::Point pos)
//{
//	cv::Mat_<float> Z(2,1); // Measurement matrix [x, y]
//	Z << pos.x, pos.y;
//	cout << " Measurement = " << endl << " " << Z << endl << endl;
//
//	////////////////////
//	//// Predict
//
//	x = (F * x);
//	cout << " 1. State = " << endl << " " << x << endl << endl;
//
//	P = F*P*F.t();
//
//	cout << " 1. Next covariance = " << endl << " " << P << endl << endl;
//
//
//	////////////////////
//	//// Correct
//
//
//	S = H * P * H.t() + R;
//
//	K = (P*H.t())*S.inv();
//	cout << " 2. Kalman Gain: " << endl << " " << K << endl << endl;
//
//	x = x + (K * (Z - (H * x)));
//	//x = x + (K_c * (Z - (H * x))); /* Constant kalman gain */
//	cout << "2. update estimate x " << endl << " " << x << endl << endl;
//
//	P = (I - K*H)*P;
//	cout << "2. update error covariance (P)" << endl << " " << P << endl << endl;
//	return x;
//}