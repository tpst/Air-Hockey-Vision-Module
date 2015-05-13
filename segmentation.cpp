#include "visModule.h"

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

	vector<vector<Point> > contours;
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