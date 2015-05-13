#include <opencv/cv.h>
#include <opencv/highgui.h>



using namespace cv;
using namespace std;


// The complete vision module class. Responsible for tracking the hockey puck (and player) and provides
// a detailed state of the puck and prediction of its location. 
class puckTracker {

public:
	//-- reference image used for detecting puck
	Mat ref;
	//-- current frame - this gets processed
	Mat frame;
	Mat frame_colour;
	//-- artificial scene
	Mat scene;

	VideoCapture cap;

	//-- region of interest ie the green part of table
	Rect roi;
	Mat tform; 

	//-- bool for testing at home. if true, open video file, else open webcam.
	bool video;
	bool debugmode;
	//-- do we need to calibrate the camera?
	bool calibrated;

	//-- output from camera calibration. 
	Mat cameraMatrix;
	Mat distCoeffs;

	// stuct that keeps track of puck variables
	struct Puck {
		Point2d position;
		Point2d last_position;
		double velocity; 
		double duration; // time between puck @ last_position and puck @ current position
		double last_duration;	
		bool flag; // 1 if it is moving towards robot

	};

	// prediction struct sent to robot when the puck is heading in that direction.
	struct Prediction {
		double distance; // along goal end w.r.t robot. 
		double angle; // incoming angle
		double eta;
		double velocity;
		double d_err;
		double a_err;
		double t_err;
		double v_err;
	};

	vector<Point2d> all_pos; 
	vector<Point2d> all_pred; // all predicted locations

	// distance travelled between two successive frames, and current speed of the puck. 
	double distance, puckSpeed;

	// variables for thresholding & morph operations. 
	int thresh, maxval;
	int size, iterations, offset;

	//-- constructor
	puckTracker(); 

	//-- calibrate the camera using a set of input images


	// main processing loop for the tracker
	void process(void);

	//-- find the puck in each frame
	int findPuck(Mat& src, Mat& img, Puck& puck);

	// validates the size and vertical position of the assumed puck object
	bool validatePuck(int size, Point2d pos);

	// does nothing yet
	void trackOpponent(vector<Point> contours, Point2d position);

	//-- Returns a frame from the video or camera
	Mat getFrame(VideoCapture& cap);

	//-- morphological operations
	Mat closeImage(Mat& src, int size, int iterations, int offset);

	Rect findROI(Mat& frame);

	double getVelocity(Puck& puck, double ratio);

	void predict(Mat& src, Point2d last, Point2d current, double velocity);
	
	bool getDirection(Puck& puck); 

};



void dilation(cv::Mat& im, int iterations, int elem, int size);
void erosion(cv::Mat& im, int iterations, int elem, int size);

vector<Point2f> calcDestPoints(vector<Point2f> source_points, vector<Point2f> dest_points);
void CallBackFunc(int event, int x, int y, int flags, void* ptr);
Mat findTransformMatrix(cv::Mat& src);