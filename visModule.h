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
	//-- current frame
	Mat frame;
	VideoCapture cap;
	//-- region of interest aka the green part of table
	Rect roi;

	//-- bool for testing at home. if true, open video file, else open webcam.
	bool video;
	bool debugmode;

	//struct puck {
	//	Point position;
	//	Point
	//};


	// Position and last known position of the hockey puck.
	Point pos, last_pos;
	vector<Point> all_pos; 

	// distance travelled between two successive frames, and current speed of the puck. 
	double distance, puckSpeed;

	// variables for thresholding & morph operations. 
	int thresh, maxval;
	int size, iterations, offset;

	//-- constructor
	puckTracker(); 

	// main processing loop for the tracker
	void process(void);

	//-- find the puck in each frame
	int findPuck(Mat& src, Mat& img);

	// validates the size and vertical position of the assumed puck object
	bool validatePuck(int size, Point2f pos);

	// does nothing yet
	void trackOpponent(vector<Point> contours, Point2f position);

	//-- Returns a frame from the video or camera
	Mat getFrame(VideoCapture& cap);

	//-- morphological operations
	Mat closeImage(Mat& src, int size, int iterations, int offset);

	void findROI(Mat& frame);

	void getVelocity(Point last_pos, Point pos, double ratio, double dur, double last_dur);

};

void predict(Mat& src, Point2f a, Point2f b);


void dilation(cv::Mat& im, int iterations, int elem, int size);
void erosion(cv::Mat& im, int iterations, int elem, int size);

vector<Point2f> calcDestPoints(vector<Point2f> source_points, vector<Point2f> dest_points);
void CallBackFunc(int event, int x, int y, int flags, void* ptr);
Mat findTransformMatrix(cv::Mat& src);