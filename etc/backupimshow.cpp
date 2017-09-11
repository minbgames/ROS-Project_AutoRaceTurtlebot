#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sherlotics/BLOCKLIGHTtoPID.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sherlotics/variable.hpp>

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <iostream>

using namespace cv;

Mat frame;
Mat lidarframe;
//---------block bar -------
Mat roi_b;
Mat hsv_b;
Mat gray_b;
Mat binWhite_b;
Mat binRed1_b;
Mat binRed2_b;
Mat binRed_b;
Mat binMerge_b;
Mat mor_b;

//----------light-----------
Mat roi_l;
Mat hsv_l;

Mat bin1_rl;
Mat bin2_rl;
Mat bin_rl;
Mat mor_rl;

Mat bin_yl;
Mat mor_yl;

//------------traffic------------
Mat roi_tr;
Mat hsv_tr;

Mat binRed1_tr;
Mat binRed2_tr;
Mat binRed_tr;
Mat binBlue_tr;

Mat bin_tr;
Mat mor_tr;

//--------------line-------------

Mat l_roi_li;
Mat r_roi_li;
Mat a_roi_li;

Mat l_hsv_li;
Mat r_hsv_li;
Mat l_gray_li;
Mat r_gray_li;

Mat l_bin_g_li;
Mat r_bin_g_li;
Mat l_bin_hsv_li;
Mat r_bin_hsv_li;
Mat l_bin_li;
Mat r_bin_li;

Mat bin_li;
Mat mor_li;

int l_b=0;
int t_b=0;
int w_b=0;
int h_b=0;
int l_rl=0;
int t_rl=0;
int w_rl=0;
int h_rl=0;
int l_yl=0;
int t_yl=0;
int w_yl=0;
int h_yl=0;
int l_tr=0;
int t_tr=0;
int w_tr=0;
int h_tr=0;
int l_x1_li=0;
int l_x2_li=0;
int l_y1_li=0;
int l_y2_li=0;
int r_x1_li=0;
int r_x2_li=0;
int r_y1_li=0;
int r_y2_li=0;

void msgCallback1(const std_msgs::Int32MultiArray::ConstPtr& imshow_msg )
{
  int Arr[4];

  int i = 0;
  for(std::vector<int>::const_iterator it = imshow_msg->data.begin(); it != imshow_msg->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}

  l_tr=Arr[0];
  t_tr=Arr[1];
  w_tr=Arr[2];
  h_tr=Arr[3];
}

void msgCallback2(const std_msgs::Int32MultiArray::ConstPtr& imshow_msg )
{
  int Arr[12];

  int i = 0;
  for(std::vector<int>::const_iterator it = imshow_msg->data.begin(); it != imshow_msg->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}
  l_b=Arr[0];
  t_b=Arr[1];
  w_b=Arr[2];
  h_b=Arr[3];
  l_rl=Arr[4];
  t_rl=Arr[5];
  w_rl=Arr[6];
  h_rl=Arr[7];
  l_yl=Arr[8];
  t_yl=Arr[9];
  w_yl=Arr[10];
  h_yl=Arr[11];
}

void msgCallback3(const std_msgs::Int32MultiArray::ConstPtr& imshow_msg )
{
  int Arr[8];

  int i = 0;
  for(std::vector<int>::const_iterator it = imshow_msg->data.begin(); it != imshow_msg->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}
  l_x1_li=Arr[0];
  l_x2_li=Arr[1];
  l_y1_li=Arr[2];
  l_y2_li=Arr[3];
  r_x1_li=Arr[4];
  r_x2_li=Arr[5];
  r_y1_li=Arr[6];
  r_y2_li=Arr[7];
}

void lidarCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    lidarframe = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  imshow("lidar_IMSHOW",lidarframe);
  waitKey(WAITKEYSIZE);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  /***********block bar*********/
  roi_b =frame(Rect(leftRect_b,topRect_b,rightRect_b-leftRect_b,bottomRect_b-topRect_b));

  cvtColor(roi_b, hsv_b, COLOR_BGR2HSV);
  cvtColor(roi_b, gray_b, COLOR_BGR2GRAY);

  inRange(gray_b, lowerWhite_b, upperwhite_b, binWhite_b);
  inRange(hsv_b, lowerRed1_b, upperRed1_b, binRed1_b);
  inRange(hsv_b, lowerRed2_b, upperRed2_b, binRed2_b);

  binRed_b = binRed1_b | binRed2_b;
  binMerge_b = binRed_b | binWhite_b; // 1_block bar

  erode(binMerge_b, mor_b, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( mor_b, mor_b, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  dilate( mor_b, mor_b, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(mor_b, mor_b, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );


  /************light*************/
  roi_l =frame(Rect(leftRect_l,topRect_l,rightRect_l-leftRect_l,bottomRect_l-topRect_l));
  cvtColor(roi_l, hsv_l, COLOR_BGR2HSV);
  //------------red light-----------

  inRange(hsv_l, lowerRed1_l, upperRed1_l, bin1_rl);
  //inRange(hsv_l, lowerRed2_l, upperRed2_l, bin2_rl);

  //bin_rl = bin1_rl | bin2_rl;
  bin_rl = bin1_rl;

  erode(bin_rl, mor_rl, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( mor_rl, mor_rl, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  dilate( mor_rl, mor_rl, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(mor_rl, mor_rl, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  //------------Green light(yellow)-----------

  inRange(hsv_l, lowerGreen_l, upperGreen_l, bin_yl);

  erode(bin_yl, mor_yl, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( mor_yl, mor_yl, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  dilate( mor_yl, mor_yl, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(mor_yl, mor_yl, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );


  /***********traffic***********/
  if(l_tr>=HALF_WIDTH){
    roi_tr =frame(Rect(leftRect_tr_1,topRect_tr_1,rightRect_tr_1-leftRect_tr_1,bottomRect_tr_1-topRect_tr_1));
  }
  else{
    roi_tr =frame(Rect(leftRect_tr_2,topRect_tr_2,rightRect_tr_2-leftRect_tr_2,bottomRect_tr_2-topRect_tr_2));

  }
  cvtColor(roi_tr, hsv_tr, COLOR_BGR2HSV);

  inRange(hsv_tr, lowerRed1_tr, upperRed1_tr, binRed1_tr);
  inRange(hsv_tr, lowerRed2_tr, upperRed2_tr, binRed2_tr);
  inRange(hsv_tr, lowerBlue_tr, upperBlue_tr, binBlue_tr);

  binRed_tr = binRed1_tr | binRed2_tr;
  bin_tr = binRed_tr | binBlue_tr;

  erode(bin_tr, mor_tr, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
	dilate( mor_tr, mor_tr, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

	//morphological closing 영역의 구멍 메우기
	dilate( mor_tr, mor_tr, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
	erode(mor_tr, mor_tr, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  /***************line*****************/

  l_roi_li = frame(leftRect_li);
  r_roi_li = frame(rightRect_li);

  if(lineVersion==GRAY_VERSION){
    cvtColor(l_roi_li, l_gray_li, COLOR_BGR2GRAY);
    cvtColor(r_roi_li, r_gray_li, COLOR_BGR2GRAY);
    inRange(l_gray_li, lowerWhite_li, upperWhite_li, l_bin_li);
    inRange(r_gray_li, lowerWhite_li, upperWhite_li, r_bin_li);
  } //gray version
  else if(lineVersion==COLOR_VERSION){
    cvtColor(l_roi_li, l_hsv_li, COLOR_BGR2HSV);
    cvtColor(r_roi_li, r_hsv_li, COLOR_BGR2HSV);
    inRange(l_hsv_li, lowerBlue_li, upperBlue_li, l_bin_li);
    inRange(r_hsv_li, lowerBlue_li, upperBlue_li, r_bin_li);
  } //color version
  else if(lineVersion==MAIN_VERSION){
    cvtColor(l_roi_li, l_hsv_li, COLOR_BGR2HSV);
    cvtColor(r_roi_li, r_gray_li, COLOR_BGR2GRAY);
    inRange(l_hsv_li, lowerYellow_li, upperYellow_li, l_bin_li);
    inRange(r_gray_li, lowerWhite_li, upperWhite_li, r_bin_li);
  } //main version



  erode(l_bin_li, l_bin_li, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( l_bin_li, l_bin_li, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  dilate( l_bin_li, l_bin_li, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(l_bin_li, l_bin_li, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  erode(r_bin_li, r_bin_li, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( r_bin_li, r_bin_li, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  dilate( r_bin_li, r_bin_li, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(r_bin_li, r_bin_li, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  //------------------------------------------------

  rectangle( frame, leftRect_li, Scalar(0,0,0),1 );
  rectangle( frame, rightRect_li, Scalar(0,0,0),1 );
  rectangle( frame, Point(leftRect_b,topRect_b), Point(rightRect_b,bottomRect_b), Scalar(255,0,0),1 );
  rectangle( frame, Point(leftRect_l,topRect_l), Point(rightRect_l,bottomRect_l), Scalar(0,0,255),1 );
  rectangle( frame, Point(leftRect_tr_1,topRect_tr_1), Point(rightRect_tr_1,bottomRect_tr_1), Scalar(0,255,0),1 );
  rectangle( frame, Point(leftRect_tr_2,topRect_tr_2), Point(rightRect_tr_2,bottomRect_tr_2), Scalar(0,255,0),1 );

  rectangle( frame, Point(l_b,t_b), Point(l_b+w_b,t_b+h_b), Scalar(255,0,0),2 ); //block bar - blue
  rectangle( frame, Point(l_rl,t_rl), Point(l_rl+w_rl,t_rl+h_rl), Scalar(0,0,255),2 ); // Light red - red
  rectangle( frame, Point(l_yl,t_yl), Point(l_yl+w_yl,t_yl+h_yl), Scalar(0,255,255),2 ); // Light yellow - yellow
  rectangle( frame, Point(l_tr,t_tr), Point(l_tr+w_tr,t_tr+h_tr), Scalar(0,255,0),2 ); // traffic - green
  line(frame, Point(l_x1_li,l_y1_li), Point(l_x2_li,l_y2_li), Scalar(0, 0, 0), 10); // l_line - black
  line(frame, Point(r_x1_li,r_y1_li), Point(r_x2_li,r_y2_li), Scalar(0, 0, 0), 10); // r_line - black

  resize( frame, frame, Size( WIDTH_SIZE, HEIGHT_SIZE ), 0, 0, CV_INTER_CUBIC );
  resize( mor_b, mor_b, Size( HALF_WIDTH, HALF_HEIGHT ), 0, 0, CV_INTER_CUBIC );
  resize( mor_yl, mor_yl, Size( HALF_WIDTH, HALF_HEIGHT ), 0, 0, CV_INTER_CUBIC );
  resize( mor_rl, mor_rl, Size( HALF_WIDTH, HALF_HEIGHT ), 0, 0, CV_INTER_CUBIC );
  resize( mor_tr, mor_tr, Size( HALF_WIDTH, HALF_HEIGHT ), 0, 0, CV_INTER_CUBIC );
  resize( l_bin_li, l_bin_li, Size( HALF_WIDTH, HALF_HEIGHT ), 0, 0, CV_INTER_CUBIC );
  resize( r_bin_li, r_bin_li, Size( HALF_WIDTH, HALF_HEIGHT ), 0, 0, CV_INTER_CUBIC );

  Mat all_view( (HEIGHT_SIZE + HALF_HEIGHT) , WIDTH_SIZE ,CV_8UC1);

  Mat merge_roi_tr(all_view, Rect(0, 0, HALF_WIDTH, HALF_HEIGHT));
  Mat merge_roi_b(all_view, Rect(HALF_WIDTH, 0, HALF_WIDTH, HALF_HEIGHT));
  Mat merge_roi_yl(all_view, Rect(0, HALF_HEIGHT, HALF_WIDTH, HALF_HEIGHT));
  Mat merge_roi_rl(all_view, Rect(HALF_WIDTH, HALF_HEIGHT, HALF_WIDTH, HALF_HEIGHT));
  Mat merge_roi_l_li(all_view, Rect(0, HEIGHT_SIZE, HALF_WIDTH, HALF_HEIGHT));
  Mat merge_roi_r_li(all_view, Rect(HALF_WIDTH, HEIGHT_SIZE, HALF_WIDTH, HALF_HEIGHT));

  mor_tr.copyTo(merge_roi_tr);
  mor_b.copyTo(merge_roi_b);
  mor_yl.copyTo(merge_roi_yl);
  mor_rl.copyTo(merge_roi_rl);
  l_bin_li.copyTo(merge_roi_l_li);
  r_bin_li.copyTo(merge_roi_r_li);

  rectangle( all_view, Rect(0, 0, HALF_WIDTH, HALF_HEIGHT), 255,2 );
  rectangle( all_view, Rect(HALF_WIDTH, 0, HALF_WIDTH, HALF_HEIGHT), 255,2 );
  rectangle( all_view, Rect(0, HALF_HEIGHT, HALF_WIDTH, HALF_HEIGHT), 255,2 );
  rectangle( all_view, Rect(HALF_WIDTH, HALF_HEIGHT, HALF_WIDTH, HALF_HEIGHT), 255,2 );
  rectangle( all_view, Rect(0, HEIGHT_SIZE, HALF_WIDTH, HALF_HEIGHT), 255,2 );
  rectangle( all_view, Rect(HALF_WIDTH, HEIGHT_SIZE, HALF_WIDTH, HALF_HEIGHT), 255,2 );

  imshow("ALL VIEW",all_view);
  imshow("ORIGINAL VIEW",frame);

  /* 표지판    차단바
     노랑신호등 빨간신호등 */

  waitKey(WAITKEYSIZE);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imshowPart_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("imshow_msg", QUEUESIZE, imageCallback);
  image_transport::Subscriber lidar_sub = it.subscribe("LIDARtoIMSHOW_msg", QUEUESIZE, lidarCallback);
  ros::Subscriber msg_sub1 = nh.subscribe("TRAFFICtoIMSHOW_msg", QUEUESIZE, msgCallback1);
  ros::Subscriber msg_sub2 = nh.subscribe("BLOCKLIGHTtoIMSHOW_msg", QUEUESIZE, msgCallback2);
  ros::Subscriber msg_sub3 = nh.subscribe("LINEtoIMSHOW_msg", QUEUESIZE, msgCallback3);

  ros::spin();

  return 0;
}
