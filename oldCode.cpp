//-- finds the puck in each frame
//-- also finds the opponent, if they're present. 
//int puckTracker::findPuck(Mat& frame, Mat& img, Puck& puck)
//{
//	vector<vector<Point>> contours;
//
//	// logic
//	bool puck_found = false;
//	bool opponent_found = false;
//
//	Mat diff;
//	Mat dst;
//
//
//	
//	int distance = 0; // minimum distance from user goal before an item can be considered as the puck
//	int puck_index = 0;
//	int opponent_index = 0;
//	double area = 0;
//
//	absdiff(frame, img, diff);
//	
//	imshow("absdiff", diff);
//	//cvtColor(diff, diff, CV_BGR2GRAY);
//
//	threshold(diff, dst, thresh, 255, THRESH_BINARY);
//	
//	// now need to sort 'blobs'
//	dst = closeImage(dst, size, iterations, offset);
//
//	if(debugmode) imshow("diff", dst);
//
//	findContours(dst.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//	
//	// object(s) have been found. Get information about each contour
//	if (contours.size() > 0)
//	{
//		///// Find the convex hull object for each contour
//		//vector<vector<Point> >hull(contours.size());
//		//for (int i = 0; i < contours.size(); i++)
//		//{
//		//	convexHull(Mat(contours[i]), hull[i], false);
//		//}
//
//		//// moments
//		//vector<Moments> mu(contours.size());
//		//for (int i = 0; i < contours.size(); i++)
//		//{
//		//	mu[i] = moments(hull[i], false);
//		//}
//
//		/////  Get the mass centers:
//		//vector<Point2f> mc(contours.size());
//		//for (int i = 0; i < contours.size(); i++)
//		//{
//		//	mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
//		//}
//		//
//		//// draw result
//		//for (int i = 0; i < contours.size(); i++)
//		//{
//		//	drawContours(dst, hull, i, Scalar(0, 0, 255), 2, 8);
//		//	circle(dst, mc[i], 3, Scalar(0, 255, 0), -1, 8); // draws centre of mass 
//		//}
//
//		/// Find the convex hull object for each contour
//		vector<vector<Point> >hull(contours.size());
//		// moments
//		vector<Moments> mu(contours.size());
//		// mass centers
//		vector<Point2f> mc(contours.size());
//
//		for (int i = 0; i < contours.size(); i++)
//		{
//			convexHull(Mat(contours[i]), hull[i], false);
//			mu[i] = moments(hull[i], false);
//			mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
//			drawContours(dst, hull, i, Scalar(0, 0, 255), 2, 8);
//		}
//
//		// if there is more than one, need to distinguish the puck from opponent
//		if(contours.size() == 1) 
//		{
//			if(validatePuck(contours[0].size(), mc[0], frame)) 
//			{
//				puck_found = true;
//			} else {
//				opponent_found = true;
//				opponent_index = 0;	
//			}
//		} else {
//			for(int i = 0; i < contours.size(); i++)
//			{
//				if(validatePuck(contours[i].size(), mc[i], frame)) 
//				{
//						puck_index = i;
//						puck_found = true;
//				}
//			}
//		}
//	
//		if(opponent_found) 
//		{
//			trackOpponent(contours[opponent_index], mc[opponent_index]);
//			circle(frame, mc[opponent_index], 8, Scalar(0, 0, 0), -3, 8);
//		}
//
//		if(puck_found) 
//		{
//			puck.position = mc[puck_index]; // set global puck position variable
//			circle(frame, mc[puck_index], 8, Scalar(255, 255, 255), -3, 8);
//		}
//	}
//	return puck_found;
//}

	//opponent_found = true;
	//		// more than one object exists
	//		// now we need to sort each contour
	//		// -- lowest vertical object will be the puck
	//		for (int i = 0; i < contours.size(); i++)
	//		{
	//			int d = mc[i].x; // horizontal distance 
	//			int a = contours[i].size(); // area
	//			if (d > distance)
	//			{
	//				distance = d;
	//				if(validatePuck(a, mc[i], frame)) puck_index = i; // make sure it is correct size. 
	//				//puck_index = i;
	//			}
	//			// the opponent will *most of the time* be the largest object
	//			if (a > area)
	//			{
	//				area = a;
	//				opponent_index = i;
	//			}
	//		}
	//		// if the puck is the largest index, adjust index of opponent. could also choose 2nd largest index here to be more accurate
	//		if(opponent_index == puck_index) 
	//		{
	//			opponent_index = 0;
	//			while(opponent_index == puck_index) 
	//			{
	//				opponent_index++;
	//			}
	//		}
	//		if(validatePuck(contours[puck_index].size(), mc[puck_index], frame)) 
	//		{
	//			puck_found = true;
	//		}
	//		
	//	}