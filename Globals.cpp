#include "Globals.h"

//Matrix to store each frame of webcame feed
Mat tempFrame1, tempFrame2, scene, tempScene;

//Region if interest boundary
Rect roi;

// HSV thresholds for puck detection

int h_min = 0;   //0
int h_max = 25; //42
int s_min = 49;   //230
int s_max = 255; //255
int v_min = 214;  //155
int v_max = 255; //255