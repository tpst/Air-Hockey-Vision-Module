#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/array.hpp>

#include "tracking.h"

#define VISION_PORT 5003

using namespace cv;
using namespace std;
using boost::asio::ip::tcp;

// The complete vision module class. Responsible for tracking the hockey puck (and player) and provides
// a detailed state of the puck and prediction of its location. 
class puckTracker {

public:
	//-- reference image used for detecting puck
	Mat ref;
	//-- current frame - this gets processed
	Mat frame;
	Mat frame_colour;

	VideoCapture cap;

	//-- region of interest ie the green part of table
	Rect roi;
	Mat tform; 

	//-- bool for testing at home. if true, open video file, else open webcam.
	bool video;
	bool debugmode;
	//-- do we need to calibrate the camera?
	bool calibrated;
	//-- YES if communicating with robot, NO if vision is run standalone
	bool talking; 

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
		double x_err;
		double y_err;
		double v_err;

	};

	// prediction struct sent to robot when the puck is heading in that direction.
	struct Prediction {
		double distance; // along goal end w.r.t robot. 
		double angle; // incoming angle
		double eta;
		double velocity;
		double d_err; // distance error
		double a_err; // angle error
		double t_err; // time error
		double v_err; // velocity error
	};

	double ratio; // ratio of pixels to m
	float mmratio;

	vector<Point2d> all_pos; 
	vector<Point2d> all_pred; // all predicted locations

	// variables for thresholding & morph operations. 
	int thresh, maxval;
	int size, iterations, offset;
	// structuring element for findPuck
	Mat element;

	//-- constructor
	puckTracker(); 

	//-- calibrate the camera using a set of input images

	// main processing loop for the tracker
	void process(void);

	//-- find the puck in each frame
	int findPuck(Mat& src, Mat& img, Puck& puck);

	// validates the size and vertical position of the assumed puck object
	bool validatePuck(double contourArea, Point2d pos, Mat& frame);

	// does nothing yet
	void trackOpponent(vector<Point> contours, Point2d position);

	//-- Returns a frame from the video or camera
	Mat getFrame(VideoCapture& cap);

	//-- morphological operations
	Mat closeImage(Mat& src, int size, int iterations, int offset);

	Rect findROI(Mat& frame);

	double getVelocity(Puck& puck, double ratio);

	bool predict(Mat& src, Point2d last, Point2d current, double velocity, string& result);
	
	bool getDirection(Puck& puck); 

	string constructMessage(Puck puck);

	string constructMessage(Prediction p);

	Mat findTransformationMatrix(cv::Mat& src, Rect& _roi);

};

void dilation(cv::Mat& im, int iterations, int elem, int size);
void erosion(cv::Mat& im, int iterations, int elem, int size);

vector<Point2f> calcDestPoints(vector<Point2f> source_points, vector<Point2f> dest_points);
void CallBackFunc(int event, int x, int y, int flags, void* ptr);
Mat findTransformMatrix(cv::Mat& src);