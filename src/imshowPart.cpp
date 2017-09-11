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

  rectangle( frame, leftRect_li, Scalar(0,0,0),1 );
  rectangle( frame, rightRect_li, Scalar(0,0,0),1 );
  rectangle( frame, Point(leftRect_b,topRect_b), Point(rightRect_b,bottomRect_b), Scalar(0,0,0),1 );
  rectangle( frame, Point(leftRect_l,topRect_l), Point(rightRect_l,bottomRect_l), Scalar(0,0,255),1 );
  rectangle( frame, Point(leftRect_tr_1,topRect_tr_1), Point(rightRect_tr_1,bottomRect_tr_1), Scalar(255,0,0),1 );
  rectangle( frame, Point(leftRect_tr_2,topRect_tr_2), Point(rightRect_tr_2,bottomRect_tr_2), Scalar(255,0,0),1 );

  rectangle( frame, Point(l_b,t_b), Point(l_b+w_b,t_b+h_b), Scalar(0,0,0),2 ); //block bar
  rectangle( frame, Point(l_rl,t_rl), Point(l_rl+w_rl,t_rl+h_rl), Scalar(0,0,255),2 ); // Light red - red
  rectangle( frame, Point(l_yl,t_yl), Point(l_yl+w_yl,t_yl+h_yl), Scalar(0,255,0),2 ); // Light green - green
  rectangle( frame, Point(l_tr,t_tr), Point(l_tr+w_tr,t_tr+h_tr), Scalar(255,0,0),3 ); // traffic - blue
  line(frame, Point(l_x1_li,l_y1_li), Point(l_x2_li,l_y2_li), Scalar(0, 0, 0), 10); // l_line - black
  line(frame, Point(r_x1_li,r_y1_li), Point(r_x2_li,r_y2_li), Scalar(0, 0, 0), 10); // r_line - black

  resize( frame, frame, Size( WIDTH_SIZE, HEIGHT_SIZE ), 0, 0, CV_INTER_CUBIC );

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
