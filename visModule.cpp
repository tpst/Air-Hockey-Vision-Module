#include "visModule.h"


// Some OS specific timer stuff /////////////////////////////
#if defined(_MSC_VER) || defined(WIN32)  || defined(_WIN32) || defined(__WIN32__) \
	|| defined(WIN64) || defined(_WIN64) || defined(__WIN64__)

#include <windows.h>
bool _qpcInited = false;
double PCFreq = 0.0;
__int64 CounterStart = 0;
void InitCounter()
{
	LARGE_INTEGER li;
	if (!QueryPerformanceFrequency(&li))
	{
		std::cout << "QueryPerformanceFrequency failed!\n";
	}
	PCFreq = double(li.QuadPart) / 1000.0f;
	_qpcInited = true;
}
double CLOCK()
{
	if (!_qpcInited) InitCounter();
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	return double(li.QuadPart) / PCFreq;
}

#endif

#if defined(unix)        || defined(__unix)      || defined(__unix__) \
	|| defined(linux) || defined(__linux) || defined(__linux__) \
	|| defined(sun) || defined(__sun) \
	|| defined(BSD) || defined(__OpenBSD__) || defined(__NetBSD__) \
	|| defined(__FreeBSD__) || defined __DragonFly__ \
	|| defined(sgi) || defined(__sgi) \
	|| defined(__MACOSX__) || defined(__APPLE__) \
	|| defined(__CYGWIN__)
double CLOCK()
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (t.tv_sec * 1000) + (t.tv_nsec*1e-6);
}
#endif

double _avgdur = 0;
double _fpsstart = 0;
double _avgfps = 0;
double _fps1sec = 0;

double avgdur(double newdur)
{
	_avgdur = 0.98*_avgdur + 0.02*newdur;
	return _avgdur;
}

double avgfps()
{
	if (CLOCK() - _fpsstart>1000)
	{
		_fpsstart = CLOCK();
		_avgfps = 0.7*_avgfps + 0.3*_fps1sec;
		_fps1sec = 0;
	}
	_fps1sec++;
	return _avgfps;
}
//////////////////////////////////////////////////////////////////


//-- constructor for pucktracker class
//   sets default variables for thresholding functions, some logic
puckTracker::puckTracker(void)
{
	video = true;
	debugmode = true;
	
	thresh = 32;
	maxval = 255;
	size = 3;
	iterations = 8;
	offset = 6;

	//-- initialise video or camera
	if (video)
	{
		ref = imread("puck_snap.png");

		cap.open("puck_test2.avi");

	} else {
		// take snapshot from camera
		// open camera item
	}
	if (!cap.isOpened()){
		cout << "Error opening video feed" << endl;
		exit(-1);
	}

}

// Process loop. 
void puckTracker::process(void)
{
	Puck puck;

	double dur = 0, last_dur = 0; // time taken for processing loop. used to find puck velocity

	namedWindow("frame", CV_WINDOW_NORMAL); // input frame

	//if (debugmode == true)
	//{
	//	//namedWindow("frame", CV_WINDOW_NORMAL); // input frame
	//	namedWindow("diff", CV_WINDOW_NORMAL); // segmentation output
	//	createTrackbar("thresh", "diff", &thresh, 255);
	//	createTrackbar("size", "diff", &size, 9);
	//	createTrackbar("iterations", "diff", &iterations, 10);
	//	createTrackbar("offset", "diff", &offset, 10);
	//}

	//Correct perspective
	Mat tform = (Mat_<float>(3, 3) << 0.7960837956561635, 0.201906957512314, 28.5334730127201,
		-0.02787305336493078, 1.145227705341308, 16.96726799282316,
		-6.150762080478036*pow(10, -5), 0.0006784661447038081, 1);
	
	// find the region of interest ie the table
	frame = getFrame(cap);

	//Mat tform = findTransformMatrix(frame);

	warpPerspective(ref, ref, tform, ref.size());

	warpPerspective(frame, frame, tform, frame.size());

	findROI(frame);

	puck.last_position = Point(-1,-1); // initialise temporary position point of the puck. 

	////-- Main Process Loop

	while (waitKey(1) != 32)
	{	
		// --timer stuff
		double start = CLOCK();

		//////////////////
		
		frame = getFrame(cap); // capture frame from input source

		if (debugmode) warpPerspective(frame, frame, tform, frame.size());

		Mat table = frame(roi); // this will be the cropped image that gets processed
		
		scene = Mat::zeros(table.size(), CV_8UC3);

		double ratio = table.cols / 1.1f; // 110 cm is the table width. ratio is the ratio of pixel to cm.

		if (findPuck(table, ref(roi), puck)) // finds the puck in the frame, fills variables in pucktracker class
		{
			circle(scene, puck.position, 12, Scalar(0, 255, 0), -2, 8);
			if (puck.last_position == Point(-1, -1)) // if this is the first recent instance of the puck being found
			{
				puck.duration = CLOCK();
				puck.last_position = puck.position;
			}
			else {
				puck.last_duration = puck.duration;
				puck.duration = CLOCK();
				getVelocity(puck, ratio);
				predict(table, puck.last_position, puck.position);
				puck.last_position = puck.position;
			}
			all_pos.push_back(puck.position); // store all known positions of the puck
		}
		else {
			puck.last_position = Point(-1, -1);
		}
		imshow("frame", table);
		imshow("scene", scene);
	}

}

void puckTracker::getVelocity(Puck& puck, double ratio)
{
	double distance = sqrt((puck.position.x - puck.last_position.x)*(puck.position.x - puck.last_position.x) + (puck.position.y - puck.last_position.y)*(puck.position.y - puck.last_position.y));
	double time = puck.duration - puck.last_duration;
	distance = distance / ratio; // convert to cm
	time = time*0.001f; // convert to s
	puck.velocity = distance / time;
	cout << "v : " << puck.velocity << " m/s" << endl;

}

//-- finds the puck in each frame
//-- also finds the opponent, if they're present. 
int puckTracker::findPuck(Mat& frame, Mat& img, Puck& puck)
{
	vector<vector<Point>> contours;

	// logic
	bool puck_found = false;
	bool opponent_found = false;

	Mat diff;
	Mat dst;

	//Artificial display of environment
	Mat scene = Mat::zeros(frame.size(), CV_8UC3);
	
	int distance = 0; // minimum distance from user goal before an item can be considered as the puck
	int puck_index = 0;
	int opponent_index = 0;
	double area = 0;

	absdiff(frame, img, diff);
	
	cvtColor(diff, diff, CV_BGR2GRAY);

	threshold(diff, dst, thresh, 255, THRESH_BINARY);
	
	// now need to sort 'blobs'
	dst = closeImage(dst, size, iterations, offset);
	//if(debugmode) imshow("diff", dst);

	findContours(dst.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	
	// object(s) have been found. Get information about each contour
	if (contours.size() > 0)
	{
		/// Find the convex hull object for each contour
		vector<vector<Point> >hull(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			convexHull(Mat(contours[i]), hull[i], false);
		}

		// moments
		vector<Moments> mu(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			mu[i] = moments(hull[i], false);
		}

		///  Get the mass centers:
		vector<Point2f> mc(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}
		
		// draw result
		for (int i = 0; i < contours.size(); i++)
		{
			drawContours(dst, hull, i, Scalar(0, 0, 255), 2, 8);
			circle(dst, mc[i], 3, Scalar(0, 255, 0), -1, 8); // draws centre of mass 
		}

		// if there is more than one, need to distinguish the puck from opponent
		if(contours.size() == 1) 
		{
			if(validatePuck(contours[0].size(), mc[0])) 
			{
				puck_found = true;
			} else {
				opponent_found = true;
				opponent_index = 0;	
			}
		} else {
			opponent_found = true;
			// more than one object exists
			// now we need to sort each contour
			// -- lowest vertical object will be the puck
			for (int i = 0; i < contours.size(); i++)
			{
				double d = mc[i].y; // vertical distance 
				double a = contours[i].size(); // area
				if (d > distance)
				{
					distance = d;
					puck_index = i;
				}
				// the opponent will *most of the time* be the largest object
				if (a > area)
				{
					area = a;
					opponent_index = i;
				}
			}
			// if the puck is the largest index, adjust index of opponent. could also choose 2nd largest index here to be more accurate
			if(opponent_index == puck_index) 
			{
				opponent_index = 0;
				while(opponent_index == puck_index) 
				{
					opponent_index++;
				}
			}
			if(validatePuck(contours[puck_index].size(), mc[puck_index])) 
			{
				puck_found = true;
			}
			
		}
	
		if(opponent_found) 
		{
			trackOpponent(contours[opponent_index], mc[opponent_index]);
			circle(frame, mc[opponent_index], 12, Scalar(0, 255, 0), -3, 8);

		}

		if(puck_found) 
		{
			puck.position = mc[puck_index]; // set global puck position variable

			circle(frame, mc[puck_index], 12, Scalar(255, 0, 0), -3, 8);
		}
	}
	return puck_found;
}

bool puckTracker::validatePuck(int size, Point2f pos) 
{
	bool valid = false;

	if (pos.y > frame.rows*0.125f) // check vertical position of puck - must be above first .125 of table.
	{
		if(size < 150) 
		{
			valid = true;
		}
	}
	return valid;
}

void puckTracker::trackOpponent(vector<Point> contours, Point2f position)
{
	// perhaps a histogram of strike position (peak vertical point in contours?)
}

Mat puckTracker::getFrame(VideoCapture& cap)
{
	Mat frame;

	bool bSuccess = cap.read(frame);

	if (!bSuccess)
	{
		cout << "Cannot get frame from stream" << endl;
		exit(-1);
	}
	return frame;
}
	
void predict(Mat& src, Point2f a, Point2f b)
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
		predict_y = src.rows;
	}
	
	predict_x = (predict_y-b.y)/slope + b.x;

	/*cout << "Previous position: " << Point(a.x, a.y) << endl;
	cout << "current position: "  << Point(b.x, b.y) << endl;
	cout << "Prediction at y = 0: " << Point(predict_x, predict_y) << endl;*/

	if((predict_x<0)||(predict_x>(src.cols+1)))
	{
		//Which side?
		if(predict_x<0)
		{
			//Left side
			bounce_x = 1;
			bounce_y = (bounce_x - b.x)*slope + b.y;

			//2nd bounce
			bounce2_x = src.cols;
			bounce2_y = (bounce2_x - bounce_x)*-slope + bounce_y;
		} 
		else 
		{
			bounce_x = src.cols;
			bounce_y = (bounce_x - b.x)*slope + b.y;
		
			//2nd bounce
			bounce2_x = 1;
			bounce2_y = (bounce2_x - bounce_x)*-slope + bounce_y;
		}
		//circle(tempScene, Point(bounce_x, bounce_y), 20, Scalar(0,0,255), 2, 8, 0);
		line(src, Point(bounce_x, bounce_y), Point(bounce2_x, bounce2_y), Scalar(0,255,0), 2, 8, 0);
		
		//Make new prediction from bounce
		predict_y2 = 0;
		predict_x2 = (predict_y2-bounce2_y)/-slope + bounce2_x;

		if(predict_y==0) 
		{
			circle(src, Point(predict_x2, predict_y2), 20, Scalar(0,0,255), 2, 8, 0);
		}
	}

	line(src, b, Point(predict_x, predict_y), Scalar(0,255,0), 2, 8, 0);
	circle(src, Point(predict_x, predict_y), 20, Scalar(0,0,255), 2, 8, 0);

}

// below is used for finding the transformation matrix for a specific camera setup. 
Mat findTransformMatrix(cv::Mat& src) 
{
	// cv::Point2f source_points[4];
	 vector<Point2f> source_points;
     //cv::Point2f dest_points[4];
	 vector<Point2f> dest_points;
	 cv::Point2f p;
	 cv::Mat transform_matrix; //Temporary matrix
	 Point2f inputQuad[4];
	 Point2f outputQuad[4];

	 //Create a window
     namedWindow("Please select 4 grid coordinates", 1);
	 //show the image
     imshow("Please select 4 grid coordinates", src);
      //set the callback function for any mouse event
	 setMouseCallback("Please select 4 grid coordinates", CallBackFunc, (void*)&source_points);

     // Wait until user press some key
     waitKey(0);

	 for (int i = 0; i<4; i++) 
	 {
		 
		 cout << "source_points["<< i <<"] == (" << source_points[i].x << ", " << source_points[i].y << ")" << endl;

	 }

	 for(int i = 0; i < 4; i++)
	 {
		 inputQuad[i] = source_points[i];
		 cout << "inputQuad: " << inputQuad[i] << endl;
	 }
	 dest_points = calcDestPoints(source_points, dest_points);
	 
	 for(int j = 0; j < 4; j++)
	 {
		 cout << "Dest point: " << dest_points[j] << endl;
		 outputQuad[j] = dest_points[j];
	 }
	 transform_matrix = cv::getPerspectiveTransform(source_points, dest_points);
	 return transform_matrix;
}

//Mouse callback function for image transformation
void CallBackFunc(int event, int x, int y, int flags, void* ptr)
{
    //Point2f *p = (Point2f*)ptr;
	vector<Point2f> *p = static_cast<vector<Point2f> *>(ptr);

	static int i = 0;
	//i = 0;
	if  ( event == EVENT_LBUTTONDOWN )
	{
		//p[i].x = x;
		//p[i].y = y;
		p->push_back(Point2f(x,y));

	//	cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		//cout << "Point is (" << p[i].x << ", " << p[i].y << ")" << endl;
	//	cout << "Point is (" << p[i][0] << ", " << p[i][1] << ")" << endl;

		cout << "i is: " << i << endl;

		//calcDestPoints(x, y, i, dest_points);
		if (++i % 4 == 0) 
		{
			cout << "Press any key to continue" << endl;
			destroyWindow("Please select 4 grid coordinates");

		}
	}
}

vector<Point2f> calcDestPoints(vector<Point2f> source_points, vector<Point2f> dest_points) 
{
	int scale = 130; 
	for (int i = 0; i < 4; i++) 
	{
		if ( i == 0 ) 
		{ 

			dest_points.push_back(source_points[i]);
			cout << "The destination co-ord for point" << i + 1 << " is (" << dest_points[i].x << ", " << dest_points[i].y << ")" << endl;

		}
		else if ( i == 1 ) 
		{
			dest_points.push_back(Point2f(source_points[0].x + scale, source_points[0].y));

			cout << "The destination co-ord for point" << i + 1 << " is (" << dest_points[i].x << ", " << dest_points[i].y << ")" << endl;
		}
		else if ( i == 2 ) 
		{
			dest_points.push_back(Point2f(source_points[0].x + scale, source_points[0].y-scale));

			cout << "The destination co-ord for point" << i + 1 << " is (" << dest_points[i].x << ", " << dest_points[i].y << ")" << endl;
		}
		else if ( i == 3)
		{
			dest_points.push_back(Point2f(source_points[0].x, source_points[0].y-scale));

			//dest_points[i].x = source_points[0].x;
			//dest_points[i].y = source_points[0].y - scale;
			cout << "The destination co-ord for point" << i + 1 << " is (" << dest_points[i].x << ", " << dest_points[i].y << ")" << endl;
		}
	}
	return dest_points;

}