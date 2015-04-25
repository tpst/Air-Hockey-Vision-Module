using namespace cv;

//Captures next frame from stream, returns image
Mat getFrame(VideoCapture& cap);

Rect findROI(cv::Mat& src);

int findPuck(cv::Mat& src, Point2f &pos);

void Dilation(cv::Mat& im, int iterations, int elem, int size);

void Erosion(cv::Mat& im, int iterations, int elem, int size);

void predict(Point2f a, Point2f b);


