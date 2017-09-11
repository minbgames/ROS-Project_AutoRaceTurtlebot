#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sherlotics/variable.hpp>

#define RESIZED_SIZE 100

using namespace cv;
using namespace std;

Mat image_raw;
Mat img_gray;
Mat img_hsv;
Mat img_binary;
Mat img_binary_gray;

int LowH = 170;
int HighH = 179;

int LowS = 50;
int HighS = 255;

int LowV = 0;
int HighV = 255;

int LowGray = 0;
int HighGray = 255;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image_raw = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  cvtColor(image_raw, img_hsv, COLOR_BGR2HSV);
  cvtColor(image_raw, img_gray, COLOR_BGR2GRAY);

  //지정한 HSV 범위를 이용하여 영상을 이진화
  inRange(img_hsv, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), img_binary);
  inRange(img_gray, Scalar(LowGray), Scalar(HighGray), img_binary_gray);


  //morphological opening 작은 점들을 제거
  erode(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(img_binary_gray, img_binary_gray, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( img_binary_gray, img_binary_gray, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  //morphological closing 영역의 구멍 메우기
  dilate( img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(img_binary, img_binary, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( img_binary_gray, img_binary_gray, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(img_binary_gray, img_binary_gray, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  //라벨링
  Mat img_labels,stats, centroids;
  int numOfLables = connectedComponentsWithStats(img_binary, img_labels,
                                             stats, centroids, 8,CV_32S);

  //영역박스 그리기
  int max = -1, idx=0;
  for (int j = 1; j < numOfLables; j++) {
      int area = stats.at<int>(j, CC_STAT_AREA);
      if ( max < area )
      {
          max = area;
          idx = j;
      }
  }



  int left = stats.at<int>(idx, CC_STAT_LEFT);
  int top  = stats.at<int>(idx, CC_STAT_TOP);
  int width = stats.at<int>(idx, CC_STAT_WIDTH);
  int height  = stats.at<int>(idx, CC_STAT_HEIGHT);


  rectangle( image_raw, Point(left,top), Point(left+width,top+height),
              Scalar(0,0,255),1 );

  imshow("binary_gray", img_binary_gray);
  imshow("binary", img_binary);
  imshow("original", image_raw);

  waitKey(10);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "colorPart_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image_raw", QUEUESIZE, imageCallback);

  namedWindow("color_search", CV_WINDOW_AUTOSIZE);

  createTrackbar("LowH", "color_search", &LowH, 179); //Hue (0 - 179)
  createTrackbar("HighH", "color_search", &HighH, 179);

  createTrackbar("LowS", "color_search", &LowS, 255); //Saturation (0 - 255)
  createTrackbar("HighS", "color_search", &HighS, 255);

  createTrackbar("LowV", "color_search", &LowV, 255); //Value (0 - 255)
  createTrackbar("HighV", "color_search", &HighV, 255);

  createTrackbar("LowGray", "color_search", &LowGray, 255); //Value (0 - 255)
  createTrackbar("HighGray", "color_search", &HighGray, 255);


  ros::spin();

  return 0;
}
