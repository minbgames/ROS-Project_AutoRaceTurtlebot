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
#include <iostream>

using namespace std;
using namespace cv;

/********************************CHANGE******************************/

int ShowBin_l = 0;
int ShowBin_b = 0;

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

/********************************CHANGE******************************/

Mat frame;
Mat frame_gray;
Mat hsv_frame;
Mat hsv_frame2;

Mat binary_frame_red;
Mat binary_frame_red_T;
Mat binary_frame_red1;
Mat binary_frame_red2;
Mat binary_frame_red1_T;
Mat binary_frame_red2_T;
Mat binary_frame_yellow;
Mat binary_frame_green;
Mat binary_frame_white;

Mat binary_frame_merge1;
Mat binary_frame_merge2;

Mat morphological_frame1;
Mat morphological_frame2;
Mat morphological_frame3;

Mat resized_frame;
Mat wanted_frame;
Mat draw_frame;

Mat roi;
Mat roi2;

int blockbar_ok=0;
int redlight_ok=0;
int yellowlight_ok=0;
int existence_ok=0;

float ratio=0;
float ratio2=0;
float ratio3=0;

ros::Publisher pub;
ros::Publisher imshow_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  blockbar_ok=0;
  redlight_ok=0;
  yellowlight_ok=0;
  existence_ok=0;

  try
  {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  roi =frame(Rect(leftRect_b,topRect_b,rightRect_b-leftRect_b,bottomRect_b-topRect_b));
  roi2 =frame(Rect(leftRect_l,topRect_l,rightRect_l-leftRect_l,bottomRect_l-topRect_l));

  cvtColor(roi, hsv_frame, COLOR_BGR2HSV);
  cvtColor(roi, frame_gray, COLOR_BGR2GRAY);

  cvtColor(roi2, hsv_frame2, COLOR_BGR2HSV);

  inRange(frame_gray, lowerWhite_b, upperwhite_b, binary_frame_white);
  inRange(hsv_frame, lowerRed1_b, upperRed1_b, binary_frame_red1);
  inRange(hsv_frame, lowerRed2_b, upperRed2_b, binary_frame_red2);

  binary_frame_red = binary_frame_red1 | binary_frame_red2;
  binary_frame_merge1 = binary_frame_red | binary_frame_white; // 1_block bar

  inRange(hsv_frame2, lowerRed1_l, upperRed1_l, binary_frame_red1_T);

  //inRange(hsv_frame2, lowerRed2_l, upperRed2_l, binary_frame_red2_T);

  binary_frame_red_T = binary_frame_red1_T;
  //binary_frame_red_T = binary_frame_red1_T | binary_frame_red2_T; // 2_light red

  //inRange(hsv_frame2, lowerYellow_l, upperYellow_l, binary_frame_yellow); // 3_light yellow

  inRange(hsv_frame2, lowerGreen_l, upperGreen_l, binary_frame_green); // 3_light green

  erode(binary_frame_merge1, morphological_frame1, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( morphological_frame1, morphological_frame1, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  dilate( morphological_frame1, morphological_frame1, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(morphological_frame1, morphological_frame1, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );


  erode(binary_frame_red_T, morphological_frame2, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( morphological_frame2, morphological_frame2, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  dilate( morphological_frame2, morphological_frame2, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(morphological_frame2, morphological_frame2, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );


  erode(binary_frame_green, morphological_frame3, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( morphological_frame3, morphological_frame3, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  dilate( morphological_frame3, morphological_frame3, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(morphological_frame3, morphological_frame3, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  //라벨링
  Mat img_labels1,stats1, centroids1;
  int numOfLables1 = connectedComponentsWithStats(morphological_frame1, img_labels1, stats1, centroids1, 8,CV_32S);

  //영역박스 그리기
  int max = -1, idx=0;
  for (int j = 1; j < numOfLables1; j++) {
      int area = stats1.at<int>(j, CC_STAT_AREA);
      if ( max < area )
      {
          max = area;
          idx = j;
      }
  }

  int left = stats1.at<int>(idx, CC_STAT_LEFT);
  int top  = stats1.at<int>(idx, CC_STAT_TOP);
  int width = stats1.at<int>(idx, CC_STAT_WIDTH);
  int height  = stats1.at<int>(idx, CC_STAT_HEIGHT);

  //--------------------------------------------------

  Mat img_labels2,stats2, centroids2;
  int numOfLables2 = connectedComponentsWithStats(morphological_frame2, img_labels2, stats2, centroids2, 8,CV_32S);

  //영역박스 그리기
  int max2 = -1, idx2=0;
  for (int j = 1; j < numOfLables2; j++) {
      int area2 = stats2.at<int>(j, CC_STAT_AREA);
      if ( max2 < area2 )
      {
          max2 = area2;
          idx2 = j;
      }
  }

  int left2 = stats2.at<int>(idx2, CC_STAT_LEFT);
  int top2  = stats2.at<int>(idx2, CC_STAT_TOP);
  int width2 = stats2.at<int>(idx2, CC_STAT_WIDTH);
  int height2  = stats2.at<int>(idx2, CC_STAT_HEIGHT);

//--------------------------------------------------

  Mat img_labels3,stats3, centroids3;
  int numOfLables3 = connectedComponentsWithStats(morphological_frame3, img_labels3, stats3, centroids3, 8,CV_32S);

  //영역박스 그리기
  int max3 = -1, idx3=0;
  for (int j = 1; j < numOfLables3; j++) {
      int area3 = stats3.at<int>(j, CC_STAT_AREA);
      if ( max3 < area3 )
      {
          max3 = area3;
          idx3 = j;
      }
  }

  int left3 = stats3.at<int>(idx3, CC_STAT_LEFT);
  int top3  = stats3.at<int>(idx3, CC_STAT_TOP);
  int width3 = stats3.at<int>(idx3, CC_STAT_WIDTH);
  int height3  = stats3.at<int>(idx3, CC_STAT_HEIGHT);

  //--------------------------------------------------

  ratio =(float)width/height;
  ratio2 =(float)width2/height2;
  ratio3 =(float)width3/height3;

  if(ShowBin_b){
    imshow("BlockBar_b",morphological_frame1);
  }
  if(ShowBin_l){
    imshow("Red_Light_l",morphological_frame2);
    imshow("Green_Light_l",morphological_frame3);
  }

  // cout << "width_r: " << width2 << endl;
  // cout << "height_r: " << height2 << endl;
  // cout << "ratio_r: " << ratio2 << endl;
  // cout << "width_g: " << width3 << endl;
  // cout << "height_g: " << height3 << endl;
  // cout << "ratio_g: " << ratio3 << endl;
  //---------------------condition-------------------------

   // 1_block bar 2_light red 3_light green

  if(height>BLOCKHEIGHT_MIN && height<BLOCKHEIGHT_MAX ){
    if(width>BLOCKWIDTH_MIN && width<BLOCKWIDTH_MAX ){
      if(ratio>BLOCKRATIO_MIN && ratio<BLOCKRATIO_MAX ){
        blockbar_ok=1;
        existence_ok=1;
      }
    }
  }

  if(height2>R_LIGHTHEIGHT_MIN && height2<R_LIGHTHEIGHT_MAX ){
    if(width2>R_LIGHTWIDTH_MIN && width2<R_LIGHTWIDTH_MAX ){
      if(ratio2>LIGHTRATIO_MIN && ratio2<LIGHTRATIO_MAX ){
        redlight_ok=1;
        existence_ok=2;
      }
    }
  }


  if(height3>Y_LIGHTHEIGHT_MIN && height3<Y_LIGHTHEIGHT_MAX ){
    if(width3>Y_LIGHTWIDTH_MIN && width3<Y_LIGHTWIDTH_MAX ){
      if(ratio3>LIGHTRATIO_MIN && ratio3<LIGHTRATIO_MAX ){
        yellowlight_ok=1;
        existence_ok=3;
      }
    }
  }

  if(!blockbar_ok){
    left=0, top=0, width=0, height=0;
  }
  if(!redlight_ok){
    left2=0, top2=0, width2=0, height2=0;
  }
  if(!yellowlight_ok){
    left3=0, top3=0, width3=0, height3=0;
  }
  /****************publish******************/

  std_msgs::Int32MultiArray imshow_msg;
  imshow_msg.data.clear();

  int sendData[12] = {left+leftRect_b,top+topRect_b,width,height,left2+leftRect_l,top2+topRect_l,width2,height2,left3+leftRect_l,top3+topRect_l,width3,height3};

  for (int i = 0; i < 12; i++)
	{
		imshow_msg.data.push_back(sendData[i]);
	}
  imshow_pub.publish(imshow_msg);

  /*------------------------------------------*/

  sherlotics::BLOCKLIGHTtoPID send_msg;
  send_msg.data = existence_ok;
  //ROS_INFO("send msg = %d", 1);
  pub.publish(send_msg);

  waitKey(WAITKEYSIZE);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "blockBarLightPart_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("normalMode_msg", QUEUESIZE, imageCallback);

  pub = nh.advertise<sherlotics::BLOCKLIGHTtoPID>("BLOCKLIGHTtoPID_msg", QUEUESIZE);
  imshow_pub = nh.advertise<std_msgs::Int32MultiArray>("BLOCKLIGHTtoIMSHOW_msg", QUEUESIZE);

  ros::spin();

  return 0;
}
