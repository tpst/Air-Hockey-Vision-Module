//class kalman_filter {
//
//	public:
//		kalman_filter();		// Constructor
//		// things
//		cv::Mat_<float> F; 		// Transition matrix (x, y, vx, vy)
//		cv::Mat_<float> H;		// Measurement matrix (x, y)
//		cv::Mat_<float> x;		// initial state location (x, y, vx, vy)
//	
//		cv::Mat_<float> K;		// Kalman Gain
//		cv::Mat_<float> S;		// temp
//		cv::Mat_<float> I;      // Identity
//
//		cv::Mat filter(cv::Point pos);
//	private:
//		cv::Mat_<float> K_c;
//		cv::Mat_<float> P;      // initial uncertainty		
//		cv::Mat_<float> R;		// Measurement noise covariance (x, y)
//};