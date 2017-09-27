#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>


using namespace cv;

#define MORSIZE 3
#define WAITKEYSIZE 1
#define QUEUESIZE 100

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


int robotMode = NORMAL_MODE;
int mode_flag = -1;

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

int leftRect_b=0;
int topRect_b=0;
int rightRect_b=640;
int bottomRect_b=100;

int leftRect_l=480;
int topRect_l=0;
int rightRect_l=640;
int bottomRect_l=200;

//--------traffic part-------

int leftRect_tr_1=320;
int topRect_tr_1=0;
int rightRect_tr_1=640;
int bottomRect_tr_1=200;

int leftRect_tr_2=0;
int topRect_tr_2=0;
int rightRect_tr_2=320;
int bottomRect_tr_2=200;
//----------imshow--------------

#define WIDTH_SIZE 640
#define HEIGHT_SIZE 480
#define HALF_WIDTH WIDTH_SIZE/2
#define HALF_HEIGHT HEIGHT_SIZE/2
