#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

extern Mat tempFrame1, tempFrame2, scene, tempScene;

extern Rect roi;

/************* Thresholds **************/
extern int h_min;
extern int h_max;
extern int s_min;
extern int s_max;
extern int v_min;
extern int v_max;