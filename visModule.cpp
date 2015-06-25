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
//   sets default variables for thresholding functions, some logic, camera calibration
puckTracker::puckTracker(void)
{
	video = true; // loading video
	debugmode = false;
	calibrated = true; // calibration is done already. 
	talking = false; // not communicating with robot. 

	if(calibrated == true) 
	{
		cameraMatrix = (Mat_<float>(3, 3) << 413.1620488515831, 0, 325.5774757503481,
											 0, 415.657300643115, 249.0529823718357,
											 0, 0, 1);
		distCoeffs = (Mat_<float>(5, 1) << -0.4623944646292301, 0.3170825823125478, 0.004937692934118649, 
											-0.003810017964592343, -0.1139398551683426);
	} 
	else 
	{
		// do calibration
	}

	/********* SEGMENTATION FOR FINDPUCK *******************/
	element = getStructuringElement( MORPH_RECT, Size( 5, 5 ), Point( -1, -1 ) );

	thresh = 45;
	maxval = 255;
	size = 3;
	iterations = 5;
	offset = 6;
	/*******************************************************/

	//-- initialise video or camera
	if (video)
	{
		Mat temp = imread("Resources/ref_image.png");

		cap.open("Resources/captured1.avi");

		undistort(temp, ref, cameraMatrix, distCoeffs);

		tform = findTransformationMatrix(ref, roi);

		warpPerspective(ref, ref, tform, ref.size());

		cvtColor(ref, ref, CV_BGR2GRAY);

	} 
	else 
	{
		// take snapshot from camera
		// open camera item
		cap.open(1);

		if (!cap.isOpened()){
			cout << "Error opening video feed" << endl;
			exit(-1);
		}

		// the camera is dark when it first opens. capture a few frames to let the light settle
		int count = 0;
		Mat temp;
		do {
			temp = getFrame(cap);
			count++;
		} while(count != 50);

		// correct camera distortion
		undistort(temp, ref, cameraMatrix, distCoeffs);
		
		tform = findTransformationMatrix(ref, roi);

		// square up the table in image
		warpPerspective(ref, ref, tform, ref.size());

		cvtColor(ref, ref, CV_BGR2GRAY); // convert reference to gray for absdiff, because frame will be gray too
	}
}

// Process loop. 
void puckTracker::process(void)
{
	
	Puck puck; // create instance of puck variable
	namedWindow("frame", CV_WINDOW_NORMAL); // input frame

	//************COMMUNICATION *******************************************/

		boost::asio::io_service io_service;

	    // acceptor object needs to be created to listen for new connections
	    tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), VISION_PORT)); 

	    tcp::socket socket(io_service);
		if(talking == true)
		{
			cout << "Waiting for connection.." << endl;
			acceptor.accept(socket);
			cout << "Robot has connected. " << endl;
			string connected_message = "1"; // send something so robot controller knows we have connected.
			boost::system::error_code ignored_error;
			boost::asio::write(socket, boost::asio::buffer(connected_message), ignored_error);
		}

	/*********************************************************************/

	//configure thresholding
	if (debugmode == true)
	{
		//namedWindow("frame", CV_WINDOW_NORMAL); // input frame
		namedWindow("thresholding", CV_WINDOW_NORMAL); // segmentation output
		createTrackbar("thresh", "thresholding", &thresh, 255);
	}
	int unfound_count = 0;
	puck.last_position = Point2d(-1,-1); // initialise temporary position point of the puck. 
	kalman_filter kf;

	//-- Main Process Loop
	while (waitKey(1) != 32)
	{	
		// --timer 
		double start = CLOCK();
		
		Mat temp = getFrame(cap); // capture frame from input source
		
		frame_colour = temp.clone();

		cvtColor(temp, temp, CV_BGR2GRAY);
		
		undistort(temp, frame, cameraMatrix, distCoeffs);
		
		warpPerspective(frame, frame, tform, frame.size());

		Mat table = frame(roi); // this will be the cropped image that gets processed
		
		ratio = table.rows / 1.1f; // 110 cm is the table width. ratio is the ratio of pixel to m.
		mmratio = table.rows / 1100.0f; // ratio in mm

		string message = ""; //initialise message string for robot. 

		if (findPuck(table, ref(roi), puck)) // finds the puck in the frame, fills variables in pucktracker class
		{
			cv::Mat_<float> x = kf.filter(puck.position);
			float temp1 = *x[0];
			float temp2 = *x[1];

			circle(table, Point(temp1, temp2), 12, Scalar(255,0,255), 1, 8);

			if(talking == true) // send puck information to robot. 
			{	
				message = constructMessage(puck); // puck was found, construct a message to send to the robot
				boost::system::error_code ignored_error;
				boost::asio::write(socket, boost::asio::buffer(message), ignored_error);
			}

			if (puck.last_position == Point2d(-1, -1)) // if this is the first recent instance of the puck being found
			{
				puck.duration = CLOCK();

				if(unfound_count > 10) {
					unfound_count = 0;
					// new kalman filter
				}

				puck.last_position = puck.position; // update position variables. 
			} 
			else // we have found the puck in 2 successive frames
			{
				puck.last_duration = puck.duration;
				puck.duration = CLOCK(); //update timer information

				puck.flag = getDirection(puck); // find direction

				if(puck.flag != 1) all_pred.clear(); // used for keeping track of puck predictions

				puck.velocity = getVelocity(puck, ratio);

				if(puck.velocity > 0.15) // if the puck is moving above a certain speed, predict its movement. 
				{
					predict(table, puck.last_position, puck.position, puck.velocity, message);
				}
				if(talking == true) // send prediction information to robot. 
				{				
					boost::system::error_code ignored_error;
					boost::asio::write(socket, boost::asio::buffer(message), ignored_error);
				}
				
				puck.last_position = puck.position; // update position
			}
			all_pos.push_back(puck.position); // store all known positions of the puck
		} 
		else // puck wasnt found - reset last position. 
		{
			puck.last_position = Point(-1, -1);
			unfound_count++;
			cv::Mat_<float> x = kf.filter();
			float kalmanGuessX = *x[0];
			float kalmanGuessY = *x[1];

			circle(table, Point(kalmanGuessX, kalmanGuessY), 12, Scalar(255,0,255), 1, 8);
		}

		imshow("frame", table);
	}

}

/*
 * Returns 1 if the puck is moving towards the robot, otherwise returns 0
 */
bool puckTracker::getDirection(Puck& puck) 
{
	if(puck.position.x > puck.last_position.x) return 1;
	return 0;
}

double puckTracker::getVelocity(Puck& puck, double ratio)
{
	double distance = sqrt((puck.position.x - puck.last_position.x)*(puck.position.x - puck.last_position.x) + (puck.position.y - puck.last_position.y)*(puck.position.y - puck.last_position.y));
	double time = puck.duration - puck.last_duration;
	distance = distance / ratio; // convert to m
	time = time*0.001f; // convert to s
	//cout << "v : " << puck.velocity << " m/s" << endl;
	return distance/time;
}

void puckTracker::trackOpponent(vector<Point> contours, Point2d position)
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
	
bool puckTracker::predict(Mat& src, Point2d last, Point2d current, double velocity, string& result) 
{
	double predict_x, predict_y; // x and y values of the predicted puck location

	int bounce_count = 0; // number of bounces 
	int bounce_max = 3; // maximum number of bounces calculated
	bool done = false;

	/* equation of a line: y = mx + b
	 *find the slope of the pucks trajectory & y intercept b */
	double slope = (current.y - last.y)/(current.x - last.x);
	double b = (current.y - slope * current.x);

	// find its direction
	if(current.x > last.x) 
	{
		// puck is travelling TOWARDS the robot goal - want to predict its location at goal
		predict_x = src.cols;
	} else if(current.x < last.x)
	{
		// away from robot - predict location at opponent goal
		predict_x = 0;
	} else {
		return done; //stationary or travelling vertical. Do nothing
	}

	// calculate predict_y 
	predict_y  = (slope*predict_x + b);
		
	// x = (y-y1)/m + x1
	Point2d last_point;

	do {
		double bounce_x, bounce_y; // x/y of bounce impact position

		if(last_point == Point2d(0,0)) last_point = current;

		if(predict_y < 0) 
		{
			bounce_y = 1;
		} else if (predict_y > src.rows) 
		{
			bounce_y = src.rows;
		} else {
			done = true;
			// good prediction
		}
		if(!done) 
		{
			bounce_count++;
			// find x value of bounce

			bounce_x = (bounce_y - last_point.y)/slope + last_point.x;

			// display
			line(src, last_point, Point2d(bounce_x, bounce_y), Scalar(255, 255, 255), 2, 8, 0);
			// puck travelling opposite direction now

			slope = -slope;

			//update prediction
			predict_y = (predict_x - bounce_x)*slope + bounce_y;

			// update the last point
			last_point = Point2d(bounce_x, bounce_y);
		} else {
			// weve made a good prediction that ends at one goal end. 
			line(src, last_point, Point2d(predict_x, predict_y), Scalar(255, 0, 0), 2, 8);

			if(predict_x != 0) 
			{
				all_pred.push_back(Point2d(predict_x, predict_y));
				if(all_pred.size() > 10) all_pred.erase(all_pred.begin());
				for(int i = 0; i < all_pred.size(); i++) 
				{
					circle(src, all_pred[i], 12, Scalar(255, 255, 255), -1, 8);
				}
			}

			circle(src, Point2d(predict_x, predict_y), 12, Scalar(255, 255, 255), -1, 8);

		}

	} while ((bounce_count != bounce_max) && (done != true)); //predict until we've bounced 3 times or found the end. 

	if(done && predict_x != 0) // only construct prediction if its heading towards robot goal 
	{
		Prediction p;
		// fill in prediction class to be sent to robot. 
		p.distance = predict_y - (src.rows/2);
		p.angle = atan(slope); // incoming angle in radians 
		p.eta = 0;
		p.velocity = velocity;
		p.d_err = 0;
		p.a_err = 0;
		p.t_err = 0;
		p.v_err = 0;
		result = constructMessage(p);
	}
	return done;
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
		 
		 //cout << "source_points["<< i <<"] == (" << source_points[i].x << ", " << source_points[i].y << ")" << endl;

	 }

	 for(int i = 0; i < 4; i++)
	 {
		 inputQuad[i] = source_points[i];
		 //cout << "inputQuad: " << inputQuad[i] << endl;
	 }
	 
	circle(src, source_points[0], 5, Scalar(255,0,0), -1, 8); // br
	circle(src, source_points[1], 5, Scalar(0,255,0), -1, 8); // bl
	circle(src, source_points[2], 5, Scalar(0,0,255), -1, 8); //tl
	imshow("src", src);
	waitKey(0);
	 dest_points = calcDestPoints(source_points, dest_points);
	 
	 for(int j = 0; j < 4; j++)
	 {
		 //cout << "Dest point: " << dest_points[j] << endl;
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
	int x_scale = 500; 
	int y_scale = 275;
	for (int i = 0; i < 4; i++) 
	{
		if ( i == 0 ) 
		{ 

			dest_points.push_back(source_points[i]);
			//cout << "The destination co-ord for point" << i + 1 << " is (" << dest_points[i].x << ", " << dest_points[i].y << ")" << endl;

		}
		else if ( i == 1 ) 
		{
			dest_points.push_back(Point2f(source_points[0].x + x_scale, source_points[0].y));

			//cout << "The destination co-ord for point" << i + 1 << " is (" << dest_points[i].x << ", " << dest_points[i].y << ")" << endl;
		}
		else if ( i == 2 ) 
		{
			dest_points.push_back(Point2f(source_points[0].x + x_scale, source_points[0].y-y_scale));

			//cout << "The destination co-ord for point" << i + 1 << " is (" << dest_points[i].x << ", " << dest_points[i].y << ")" << endl;
		}
		else if ( i == 3)
		{
			dest_points.push_back(Point2f(source_points[0].x, source_points[0].y-y_scale));

			//dest_points[i].x = source_points[0].x;
			//dest_points[i].y = source_points[0].y - scale;
			//cout << "The destination co-ord for point" << i + 1 << " is (" << dest_points[i].x << ", " << dest_points[i].y << ")" << endl;
		}
	}
	return dest_points;

}