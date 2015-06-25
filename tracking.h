
class kalman_filter {

	private:
		cv::Mat_<float> F; 		// Transition matrix (x, y, vx, vy)
		cv::Mat_<float> H;		// Measurement matrix (x, y)
		cv::Mat_<float> x;		// initial state location (x, y, vx, vy)
	
		cv::Mat_<float> K;		// Kalman Gain
		cv::Mat_<float> S;		// temp
		cv::Mat_<float> I;      // Identity

		// Covariances
		cv::Mat_<float> P;      // Covariance matrix (initial uncertainty)
		cv::Mat_<float> Q;		// Process noise covariance
		cv::Mat_<float> R;		// Measurement noise covariance (x, y)

	public:
		cv::Mat filter(cv::Point2d pos);
		kalman_filter(); // constructor

};


