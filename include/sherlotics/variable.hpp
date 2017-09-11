#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

#define MORSIZE 3
#define WAITKEYSIZE 1
#define QUEUESIZE 1

//------------display-------------

bool disp_angular_velocity = 0;
bool disp_linear_velocity = 0;
bool disp_obstacle = 0;
bool disp_lineGap = 0;
bool disp_error_integral = 0;
bool disp_error_total = 0;
bool disp_center_x = 0;
bool disp_pose = 0;

//-------------mode---------------

#define NORMAL_MODE 0
#define PARKING_MODE 1
#define LIDAR_MODE 2
#define NORMAL_TO_PARKING 3

int robotMode = LIDAR_MODE;
//----------PID part-----------

float goal=0;
float alpha=1;

float Kp=1;
float Ki=0;
float Kd=0;
//P, I, D setting
float cycle_time=0.1;
float error_alpha=0.7;

#define SCALE_VALUE 0.15 //0.1
#define INITIALVELOCITY 0.15
#define MAX_ERROR 4
#define MIN_ERROR -4
#define MAX_INTEGRAL 0.4
#define MIN_INTEGRAL -0.4

//--------line part-------

#define GRAY_VERSION 1
#define COLOR_VERSION 2
#define MAIN_VERSION 3

int lineVersion = GRAY_VERSION;

Scalar lowerYellow_li(0, 0, 200);
Scalar upperYellow_li(45, 100, 255);
Scalar lowerWhite_li(200);
Scalar upperWhite_li(255);

Scalar lowerBlue_li(100, 50, 0);
Scalar upperBlue_li(140, 255, 255);

int l_leftRect_li = 0;
int l_topRect_li = 360;
int l_rightRect_li = 250;
int l_bottomRect_li = 480;

int r_leftRect_li = 390;
int r_topRect_li = 360;
int r_rightRect_li = 640;
int r_bottomRect_li = 480;

Rect leftRect_li(l_leftRect_li, l_topRect_li, l_rightRect_li-l_leftRect_li, l_bottomRect_li-l_topRect_li);
Rect rightRect_li(r_leftRect_li, r_topRect_li, r_rightRect_li-r_leftRect_li, r_bottomRect_li-r_topRect_li);

//-------block light part------

#define BLOCKRATIO_MAX 9.1
#define BLOCKRATIO_MIN 5
#define BLOCKWIDTH_MAX 550
#define BLOCKWIDTH_MIN 300
#define BLOCKHEIGHT_MAX 160
#define BLOCKHEIGHT_MIN 50

#define LIGHTRATIO_MAX 1.3
#define LIGHTRATIO_MIN 0.7

#define R_LIGHTWIDTH_MAX 40
#define R_LIGHTWIDTH_MIN 10
#define R_LIGHTHEIGHT_MAX 40
#define R_LIGHTHEIGHT_MIN 10

#define Y_LIGHTWIDTH_MAX 35
#define Y_LIGHTWIDTH_MIN 5
#define Y_LIGHTHEIGHT_MAX 35
#define Y_LIGHTHEIGHT_MIN 5

#define G_LIGHTWIDTH_MAX 35
#define G_LIGHTWIDTH_MIN 5
#define G_LIGHTHEIGHT_MAX 35
#define G_LIGHTHEIGHT_MIN 5

Scalar lowerWhite_b(200);
Scalar upperwhite_b(255);
Scalar lowerRed1_b(0, 100, 0);
Scalar upperRed1_b(10, 255, 255);
Scalar lowerRed2_b(170, 100, 0);
Scalar upperRed2_b(179, 255, 255);
Scalar lowerRed1_l(10, 50, 215);
Scalar upperRed1_l(25, 100, 240);
Scalar lowerRed2_l(0, 0, 0);
Scalar upperRed2_l(0, 0, 0);
Scalar lowerYellow_l(0, 0, 0);
Scalar upperYellow_l(0, 0, 0);
Scalar lowerGreen_l(80, 40, 190);
Scalar upperGreen_l(100, 180, 255);

int leftRect_b=0;
int topRect_b=0;
int rightRect_b=640;
int bottomRect_b=100;

int leftRect_l=480;
int topRect_l=0;
int rightRect_l=640;
int bottomRect_l=200;

//--------traffic part-------
#define TRAFFICRATIO_MAX 1.4
#define TRAFFICRATIO_MIN 0.6
#define TRAFFICWIDTH_MAX 80
#define TRAFFICWIDTH_MIN 50
#define TRAFFICHEIGHT_MAX 80
#define TRAFFICHEIGHT_MIN 50

Scalar lowerBlue_tr(100, 100, 0);
Scalar upperBlue_tr(130, 255, 255);
Scalar lowerRed1_tr(0, 100, 0);
Scalar upperRed1_tr(10, 255, 255);
Scalar lowerRed2_tr(170, 100, 0);
Scalar upperRed2_tr(179, 255, 255);

int leftRect_tr_1=460;
int topRect_tr_1=0;
int rightRect_tr_1=640;
int bottomRect_tr_1=200;

int leftRect_tr_2=0;
int topRect_tr_2=0;
int rightRect_tr_2=180;
int bottomRect_tr_2=200;
//----------imshow--------------

#define WIDTH_SIZE 640
#define HEIGHT_SIZE 480
#define HALF_WIDTH WIDTH_SIZE/2
#define HALF_HEIGHT HEIGHT_SIZE/2
