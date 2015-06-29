/* 
 * Kalman filter for tracking the position of the hockey puck
 */
class puck_filter {

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
		cv::Mat filter();

		puck_filter(cv::Point2d pos); // constructor

};

class prediction_filter {

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
		cv::Mat filter();

		prediction_filter(cv::Point2d pos); // constructor
		
};