#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;
using namespace std;

#include "functions.h"
#include "globals.h"
#include "visModule.h"

void extendLine(cv::Mat *src, Point2f a, Point2f b)
{
	double slope = (b.y-a.y)/(b.x-a.x);

	Point p(0,0), q(src->cols,src->rows);

	p.y = -(a.x - p.x) * slope + a.y;
    q.y = -(b.x - q.x) * slope + b.y;

	//NOTE: THIS NEEDS TO BE UPDATED TO CONSIDER VERTICAL CASES
	if(b.x > a.x)
	{
		line(*src, b, q, Scalar(255,0,0),2,8,0);
	} else {
		line(*src, b, p, Scalar(255,0,0),2,8,0);
	}
    ///line(*src,p,q,Scalar(255,0,0),2,8,0);
}

void predict(Point2f a, Point2f b)
{
	
	double slope = (b.y-a.y)/(b.x-a.x);

	double predict_x, predict_y;
	double predict_x2, predict_y2;
	double bounce_x, bounce_y;
	double bounce2_x, bounce2_y;

	// x = (y-y1)/m + x1
	if(b.y < a.y)  //If puck is travelling upwards (w.r.t camera)
	{
		predict_y = 0;
	} else {  // else its going down
		predict_y = 458;
	}
	
	predict_x = (predict_y-b.y)/slope + b.x;

	/*cout << "Previous position: " << Point(a.x, a.y) << endl;
	cout << "current position: "  << Point(b.x, b.y) << endl;
	cout << "Prediction at y = 0: " << Point(predict_x, predict_y) << endl;*/

	if((predict_x<0)||(predict_x>468))
	{
		//Which side?
		if(predict_x<0)
		{
			//Left side
			bounce_x = 1;
			bounce_y = (bounce_x - b.x)*slope + b.y;

			//2nd bounce
			bounce2_x = 467;
			bounce2_y = (bounce2_x - bounce_x)*-slope + bounce_y;
		} 
		else 
		{
			bounce_x = 467;
			bounce_y = (bounce_x - b.x)*slope + b.y;
		
			//2nd bounce
			bounce2_x = 1;
			bounce2_y = (bounce2_x - bounce_x)*-slope + bounce_y;
		}
		//circle(tempScene, Point(bounce_x, bounce_y), 20, Scalar(0,0,255), 2, 8, 0);
		line(tempScene, Point(bounce_x, bounce_y), Point(bounce2_x, bounce2_y), Scalar(0,255,0), 1, 8, 0);
		
		//Make new prediction from bounce
		predict_y2 = 0;
		predict_x2 = (predict_y2-bounce2_y)/-slope + bounce2_x;

		if(predict_y==0) 
		{
			circle(tempScene, Point(predict_x2, predict_y2), 20, Scalar(0,0,255), 2, 8, 0);
		}
	}

	line(tempFrame1, b, Point(predict_x, predict_y), Scalar(255,0,0), 2, 8, 0);
	line(tempScene, b, Point(predict_x, predict_y), Scalar(0,255,0), 1, 8, 0);
	circle(tempScene, Point(predict_x, predict_y), 20, Scalar(0,0,255), 2, 8, 0);

	imshow("Feed", tempFrame1);
	imshow("Scene", tempScene);
}

int findPuck(cv::Mat& src, Point2f &pos)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Moments mu;
	Rect boundingRectangle = Rect(0,0,0,0);
	Mat hsv;
	//Convert to HSV color space
	cvtColor(src, hsv, COLOR_BGR2HSV);
	//Filter image
	inRange(hsv,Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), hsv);
	//Morphological operations to smooth result
	Erosion(hsv, 3, 0, 2);
	Dilation(hsv, 3, 0, 5); //These values work ok on webcam, even at decent speeds

	//Convex hull?
	imshow("Processed", hsv);

	findContours(hsv.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	if(contours.size() == 1)
	{
		vector< vector<Point> > largestContourVec;
		largestContourVec.push_back(contours.at(contours.size()-1));
		//make a bounding rectangle around the largest contour then find its centroid
		//this will be the object's final estimated position.
		boundingRectangle = boundingRect(largestContourVec.at(0));
		int xpos = boundingRectangle.x+boundingRectangle.width/2;
		int ypos = boundingRectangle.y+boundingRectangle.height/2;
		pos.x = xpos;
		pos.y = ypos;

		//Draw contours and bounding rect on our source frame
		Mat drawing = Mat::zeros( hsv.size(), CV_8UC3 );
		for( int i = 0; i< contours.size(); i++ )
		{
			drawContours( drawing, contours, i, Scalar(0,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
			rectangle(src, boundingRectangle.tl(), boundingRectangle.br(), Scalar(255,0,0), 2, 8, 0);
		}
		
		return 1;
	} else {
		return 0;
	}
	
}

Rect findROI(cv::Mat& src)
{
	cv::Mat roi = Mat::zeros( src.size(), CV_8UC3 );
	cv::Mat hsv, dst;

	int HMIN = 64;   //64
	int HMAX = 104; //104
	int SMIN = 80;   //80
	int SMAX = 230; //230
	int VMIN = 51;  //51
	int VMAX = 219; //219

	//Create trackbars to adjust HSV colouring
	namedWindow("Trackbars", CV_WINDOW_NORMAL);
	resizeWindow("Trackbars", 420, 380);

	createTrackbar( "H_MIN", "Trackbars", &HMIN, HMAX );
	createTrackbar( "H_MAX", "Trackbars", &HMAX, HMAX );
	createTrackbar( "S_MIN", "Trackbars", &SMIN, SMAX );
	createTrackbar( "S_MAX", "Trackbars", &SMAX, SMAX );
	createTrackbar( "V_MIN", "Trackbars", &VMIN, VMAX );
	createTrackbar( "V_MAX", "Trackbars", &VMAX, VMAX );	

	//Convert to HSV color space
	cvtColor(src, hsv, COLOR_BGR2HSV);

	while(true)
	{
		//Filter image
		inRange(hsv,Scalar(HMIN, SMIN, VMIN), Scalar(HMAX, SMAX, VMAX), dst);

		imshow("ROI", dst);

			//if user press 'space' key
        if (waitKey(10) == 32)
        {
			cout << "Thanks. Thresholded image stored." << endl;   
			break;
        }
	}
	vector<vector<Point> > contours;
	vector<Point> contoursAll;
	vector<Vec4i> hierarchy;
	findContours(dst.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    vector<Point> hull(1);

	for( int i = 0; i < contours.size(); i++ )
	{
		for( int j = 0; j < contours[i].size(); j++)
		{
			contoursAll.push_back(contours[i][j]);
		}
	}

    // Compute minimal bounding box
    cv::Rect box = boundingRect(cv::Mat(contoursAll));
	
	/// Draw contours + hull results
    Mat drawing = Mat::zeros( hsv.size(), CV_8UC3 );
 
	rectangle(drawing, box.tl(), box.br(), Scalar(0,255,0), 2, 8, 0);

	imshow("Drawing", drawing);
	imshow("Crop", src(box));
	waitKey(0);

	destroyAllWindows();
	return box;
}


//Captures next frame from stream, returns image
Mat getFrame(VideoCapture& cap)
{
	cv::Mat frame;
	bool bSuccess = cap.read(frame); // read a new frame from video

    if (!bSuccess) //if not success, break loop
    {
		cout << "Cannot read a frame from video stream" << endl;
		exit(-1);
	}
	return frame;
}

void Erosion(cv::Mat& im, int iterations, int elem, int size)
{
	int erosion_type;
	if( elem == 0 ){ erosion_type = MORPH_RECT; }
	else if( elem == 1 ){ erosion_type = MORPH_CROSS; }
	else if( elem == 2) { erosion_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement( erosion_type, Size( size, size ), Point(-1,-1) );
	
	erode(im, im, element, Point(-1,-1), iterations, 1);
}

void Dilation(cv::Mat& im, int iterations, int elem, int size)
{
	int erosion_type;
	if( elem == 0 ){ erosion_type = MORPH_RECT; }
	else if( elem == 1 ){ erosion_type = MORPH_CROSS; }
	else if( elem == 2) { erosion_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement( erosion_type, Size( size, size ), Point(-1,-1) );
	
	dilate(im, im, element, Point(-1,-1), iterations, 1);
}

