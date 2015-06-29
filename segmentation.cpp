#include "visModule.h"

int puckTracker::findPuck(Mat& frame, Mat& img, Puck& puck)
{
	vector<vector<Point>> contours;

	bool puck_found = false;
	bool opponent_found = false;

	int puck_index = -1;
	int opponent_index = -1;
	int robot_index = -1;

	Mat diff, dst;

	absdiff(frame, img, diff);

	threshold(diff, dst, thresh, 255, THRESH_BINARY);

	morphologyEx( dst, dst, MORPH_CLOSE, element );

	erosion( dst, 1, 0, 3);

	if(debugmode) 
	{
		imshow("absdiff", diff);
		imshow("thresholding", dst);
	}

	findContours(dst.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	Mat scene = Mat::zeros(frame.size(), CV_8UC3);

	if(contours.size() > 0)
	{
		/// Find the convex hull object for each contour
		vector<vector<Point> >hull(contours.size());
		// moments
		vector<Moments> mu(contours.size());
		// mass centers
		vector<Point2f> mc(contours.size());

		for (int i = 0; i < contours.size(); i++)
		{
			convexHull(Mat(contours[i]), hull[i], false);
			mu[i] = moments(hull[i], false);
			mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
			
			// check for connected components
			for(int j = 0; j < hull[i].size(); j++)
			{
				Point pt = hull[i][j];
				if((pt.x < frame.cols*0.05f) && (contourArea(hull[i]) > 150))
				{
					opponent_index = i;
					opponent_found = true;
				}
				if((pt.x > frame.cols*0.98f) && (contourArea(hull[i]) > 150)) 
				{
					if((pt.y > frame.rows*0.25f) && (pt.y < frame.rows*0.75f))
					{
					// this will be the robot. 
						robot_index = i;
					}
				}
			}

			drawContours(scene, hull, i, Scalar(0, 0, 255), -2, 8);

			if(i != opponent_index && i != robot_index)
			{
				if(validatePuck(contourArea(hull[i]), mc[i], frame))
				{
					puck_index = i;
					puck_found = true;
				}
			}
		}

		if(puck_found) // puck size is approx 220 - 500 depending on blur pixel. 
		{
			puck.position = mc[puck_index];
			circle(frame, mc[puck_index], 8, Scalar(255, 255, 255), -1, 8);
			circle(scene, mc[puck_index], 8, Scalar(255, 255, 255), -1, 8);
		}
		if(opponent_found) 
		{
			trackOpponent(contours[opponent_index], mc[opponent_index]);
			circle(frame, mc[opponent_index], 8, Scalar(0, 0, 0), -1, 8);
			circle(scene, mc[opponent_index], 8, Scalar(0, 255, 0), -1, 8);
		}
		//imshow("scene", scene);
	}
	return puck_found;
}

bool puckTracker::validatePuck(double contourArea, Point2d pos, Mat& frame) 
{
	bool valid = false;

	if (pos.x > frame.cols*0.05f) // check horizontal position of puck - must be above first .125 of table.
	{
		if((contourArea >= 220) && (contourArea <= 500)) 
		{
			valid = true;
		} else {
			//cout << size << endl;
		}
	}
	return valid;
}

Mat puckTracker::closeImage(Mat& src, int size, int iterations, int offset)
{
	Mat dst = src.clone();
	erosion(dst, iterations, 2, size);
	dilation(dst, iterations + offset, 2, size);
	erosion(dst, offset, 2, size);
	return dst;
}

Rect puckTracker::findROI(Mat& src)
{
	//find green,
	Mat hsv, dst;
	bool debugging = true;

	int HMIN = 57; //65
	int HMAX = 84; //199
	int SMIN = 60; //51
	int SMAX = 255; //220
	int VMIN = 0; //81
	int VMAX = 235; //247

	//Create trackbars to adjust HSV colouring
	namedWindow("Trackbars", CV_WINDOW_NORMAL);
	resizeWindow("Trackbars", 420, 380);
	createTrackbar("H_MIN", "Trackbars", &HMIN, HMAX);
	createTrackbar("H_MAX", "Trackbars", &HMAX, HMAX);
	createTrackbar("S_MIN", "Trackbars", &SMIN, SMAX);
	createTrackbar("S_MAX", "Trackbars", &SMAX, SMAX);
	createTrackbar("V_MIN", "Trackbars", &VMIN, VMAX);
	createTrackbar("V_MAX", "Trackbars", &VMAX, VMAX);

	//Convert to HSV color space
	cvtColor(src, hsv, COLOR_BGR2HSV);

	while (true)
	{
		//Filter image
		inRange(hsv, Scalar(HMIN, SMIN, VMIN), Scalar(HMAX, SMAX, VMAX), dst);

		erosion(dst, 3, 2, 3);
		dilation(dst, 3, 2, 3);

		imshow("ROI", dst);
		//if user press 'space' key
		if (waitKey(10) == 32)
		{
			break;
		}
	}

	vector<vector<Point>> contours;
	vector<Point> contoursAll;

	findContours(dst.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	for (int i = 0; i < contours.size(); i++)
	{
		for (int j = 0; j < contours[i].size(); j++)
		{
			contoursAll.push_back(contours[i][j]);
		}
	}

	// Compute minimal bounding box
	//RotatedRect box = minAreaRect(contoursAll);
	Rect roi = boundingRect(contoursAll);
	/*Point2f rect_points[4];
	box.points(rect_points);

	for (int i = 0; i < 4; i++)
	{
	line(frame, rect_points[i], rect_points[(i + 1) % 4], Scalar(0, 255, 0), 2, 8);
	}*/
	//rectangle(frame, roi, Scalar(0, 255, 0), 2, 8);
	//imshow("frame", frame);
	//waitKey(0);
	destroyAllWindows();
	return roi;
}

void erosion(cv::Mat& im, int iterations, int elem, int size)
{
	int erosion_type;
	if (elem == 0){ erosion_type = MORPH_RECT; }
	else if (elem == 1){ erosion_type = MORPH_CROSS; }
	else if (elem == 2) { erosion_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement(erosion_type, Size(size, size), Point(-1, -1));

	erode(im, im, element, Point(-1, -1), iterations, 1);
}

void dilation(cv::Mat& im, int iterations, int elem, int size)
{
	int erosion_type;
	if (elem == 0){ erosion_type = MORPH_RECT; }
	else if (elem == 1){ erosion_type = MORPH_CROSS; }
	else if (elem == 2) { erosion_type = MORPH_ELLIPSE; }

	Mat element = getStructuringElement(erosion_type, Size(size, size), Point(-1, -1));

	dilate(im, im, element, Point(-1, -1), iterations, 1);
}

Mat puckTracker::findTransformationMatrix(cv::Mat& src, Rect& _roi)
{
	vector<Point2f> source_points;
	vector<Point2f> dest_points;

	Mat hsv, dst;

	int HMIN = 57; //65
	int HMAX = 84; //199
	int SMIN = 60; //51
	int SMAX = 255; //220
	int VMIN = 0; //81
	int VMAX = 235; //247

	//Create trackbars to adjust HSV colouring
	namedWindow("Trackbars", CV_WINDOW_NORMAL);
	resizeWindow("Trackbars", 420, 380);
	createTrackbar("H_MIN", "Trackbars", &HMIN, HMAX);
	createTrackbar("H_MAX", "Trackbars", &HMAX, HMAX);
	createTrackbar("S_MIN", "Trackbars", &SMIN, SMAX);
	createTrackbar("S_MAX", "Trackbars", &SMAX, SMAX);
	createTrackbar("V_MIN", "Trackbars", &VMIN, VMAX);
	createTrackbar("V_MAX", "Trackbars", &VMAX, VMAX);

	//Convert to HSV color space
	cvtColor(src, hsv, COLOR_BGR2HSV);

	//while (true)
	//{
		//Filter image
		inRange(hsv, Scalar(HMIN, SMIN, VMIN), Scalar(HMAX, SMAX, VMAX), dst);

		erosion(dst, 3, 2, 3);
		dilation(dst, 3, 2, 3);

	//	imshow("ROI", dst);
	//	//if user press 'space' key
	//	if (waitKey(10) == 32)
	//	{
	//		break;
	//	}
	//}

	vector<vector<Point>> contours;
	vector<Point> contoursAll;

	findContours(dst.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	for (int i = 0; i < contours.size(); i++)
	{
		for (int j = 0; j < contours[i].size(); j++)
		{
			contoursAll.push_back(contours[i][j]);
		}
	}

	// Compute minimal bounding box
	RotatedRect box = minAreaRect(contoursAll);
	//Rect roi = boundingRect(contoursAll);

	// rotated rectangle
    Point2f rect_points[4]; box.points( rect_points );
   /* for( int j = 0; j < 4; j++ )
       line( src, rect_points[j], rect_points[(j+1)%4], Scalar(0,255,0), 1, 8 );*/

	//circle(src, rect_points[0], 5, Scalar(255,0,0), -1, 8); // br
	//circle(src, rect_points[1], 5, Scalar(0,255,0), -1, 8); // bl
	//circle(src, rect_points[2], 5, Scalar(0,0,255), -1, 8); //tl
	//do some reshuffling
	source_points.push_back(rect_points[1]);
	source_points.push_back(rect_points[0]);
	source_points.push_back(rect_points[3]);
	source_points.push_back(rect_points[2]);

	dest_points = calcDestPoints(source_points, dest_points);
	Mat transform_matrix = cv::getPerspectiveTransform(source_points, dest_points);
	
	warpPerspective(dst, dst, transform_matrix, dst.size());

	vector<vector<Point>> contours2;
	vector<Point> contoursAll2;

	findContours(dst.clone(), contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	for (int i = 0; i < contours2.size(); i++)
	{
		for (int j = 0; j < contours2[i].size(); j++)
		{
			contoursAll2.push_back(contours2[i][j]);
		}
	}

	_roi = boundingRect(contoursAll2);

	destroyWindow("Trackbars");

	return transform_matrix;
}