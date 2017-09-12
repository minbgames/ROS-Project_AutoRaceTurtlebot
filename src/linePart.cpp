#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sherlotics/LINEtoPID.h>
#include <sherlotics/variable.hpp>
#include <cmath>

using namespace std;

/********************************CHANGE******************************/
int ShowBin_li = 0;

Scalar lowerWhite_li(200);
Scalar upperWhite_li(255);
Scalar lowerYellow_li(0, 0, 200);
Scalar upperYellow_li(45, 100, 255);

/********************************CHANGE******************************/

Scalar lowerBlue_li(100, 50, 0);
Scalar upperBlue_li(140, 255, 255);

#define GRAY_VERSION 1
#define COLOR_VERSION 2
#define MAIN_VERSION 3

int lineVersion = GRAY_VERSION;

Mat image;
Mat leftCutImage;
Mat rightCutImage;
Mat l_hsvImage;
Mat r_hsvImage;
Mat l_grayImage;
Mat r_grayImage;
Mat grayImage;
Mat allGrayImage;
Mat l_binary_g;
Mat r_binary_g;
Mat l_binary_hsv;
Mat r_binary_hsv;
Mat l_binaryImage;
Mat r_binaryImage;
Mat binaryImage;

ros::Publisher pub;
ros::Publisher imshow_pub;

float l_x1=0,l_x2=0,l_y1=480,l_y2=480,r_x1=640,r_x2=640,r_y1=480,r_y2=480;
float l_xMul, l_yMul, r_xMul, r_yMul, center_x=320;
float  originLeft_x, originRight_x, left_x, right_x, l_descent, r_descent, line_error=0;
int leftLine_ok, rightLine_ok, l_lineOk, r_lineOk, l_line_count=0, r_line_count=0, lineSize_ok=1;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  leftCutImage = image(leftRect_li);
  rightCutImage = image(rightRect_li);

  if(lineVersion==GRAY_VERSION){
    cvtColor(leftCutImage, l_grayImage, COLOR_BGR2GRAY);
    cvtColor(rightCutImage, r_grayImage, COLOR_BGR2GRAY);
    inRange(l_grayImage, lowerWhite_li, upperWhite_li, l_binaryImage);
    inRange(r_grayImage, lowerWhite_li, upperWhite_li, r_binaryImage);
  } //gray version
  else if(lineVersion==COLOR_VERSION){
    cvtColor(leftCutImage, l_hsvImage, COLOR_BGR2HSV);
    cvtColor(rightCutImage, r_hsvImage, COLOR_BGR2HSV);
    inRange(l_hsvImage, lowerBlue_li, upperBlue_li, l_binaryImage);
    inRange(r_hsvImage, lowerBlue_li, upperBlue_li, r_binaryImage);
  } //color version
  else if(lineVersion==MAIN_VERSION){
    cvtColor(leftCutImage, l_hsvImage, COLOR_BGR2HSV);
    cvtColor(rightCutImage, r_grayImage, COLOR_BGR2GRAY);
    inRange(l_hsvImage, lowerYellow_li, upperYellow_li, l_binaryImage);
    inRange(r_grayImage, lowerWhite_li, upperWhite_li, r_binaryImage);
  }


  cvtColor(image, allGrayImage, COLOR_BGR2GRAY);
  inRange(allGrayImage, 0, 0, binaryImage);

  Size szL = l_binaryImage.size();
  Size szR = r_binaryImage.size();

  Mat SubMatL(binaryImage, leftRect_li);
  l_binaryImage.copyTo(SubMatL);
  Mat SubMatR(binaryImage, rightRect_li);
  r_binaryImage.copyTo(SubMatR);

  erode(binaryImage, binaryImage, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( binaryImage, binaryImage, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  //morphological closing 영역의 구멍 메우기
  dilate( binaryImage, binaryImage, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(binaryImage, binaryImage, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  if(ShowBin_li){
      imshow("LineBin_li",binaryImage);
  }

  vector<Vec4i> lines;
  HoughLinesP(binaryImage, lines, 1, CV_PI / 180, 10, 0, 3);
  for (int i = 0; i < lines.size(); i++) {
      Vec4i L = lines[i];
  }

  Vec4i params;
  int x1[500], y1[500], x2[500], y2[500];
  int l_x1_sum=0, l_y1_sum=0, l_x2_sum=0, l_y2_sum=0, r_x1_sum=0, r_y1_sum=0, r_x2_sum=0, r_y2_sum=0;

  cout << "linesize: " << lines.size() << endl;

  if(lines.size() > 200) lineSize_ok=0;
  else lineSize_ok=1;

  l_line_count = 0;
  r_line_count = 0;

  if(lineSize_ok){
    for (int k =0; k < lines.size(); k++) {
        params = lines[k];
        x1[k] = params[0];
        y1[k] = params[1];
        x2[k] = params[2];
        y2[k] = params[3];

        // line(image, Point(x1[k], y1[k]), Point(x2[k], y2[k]), Scalar(0, 0, 255), 2);
        if ((x1[k] <= l_rightRect_li) && (x2[k] <= l_rightRect_li)) {
            if ((y1[k] >= l_topRect_li) && (y2[k] >= l_topRect_li)) {
              l_xMul=x1[k]-x2[k];
              l_yMul=y1[k]-y2[k];
              l_lineOk= l_xMul*l_yMul;

              if(l_lineOk<0){
                l_x1_sum += x1[k];
                l_x2_sum += x2[k];
                l_y1_sum += y1[k];
                l_y2_sum += y2[k];
                l_line_count++;
              }
            }
        }

        if ((x1[k] >= r_leftRect_li) && (x2[k] >= r_leftRect_li)) {
            if ((y1[k] >= r_topRect_li) && (y2[k] >= r_topRect_li)) {
              r_xMul=x1[k]-x2[k];
              r_yMul=y1[k]-y2[k];
              r_lineOk= r_xMul*r_yMul;

              if(r_lineOk>0){
                  r_x1_sum += x1[k];
                  r_x2_sum += x2[k];
                  r_y1_sum += y1[k];
                  r_y2_sum += y2[k];
                  r_line_count++;
              }
            }
        }
    }
  }

  leftLine_ok=0;
  rightLine_ok=0;

  if(l_line_count>10){
      leftLine_ok=1;
      l_x1 = l_x1_sum / l_line_count;
      l_x2 = l_x2_sum / l_line_count;
      l_y1 = l_y1_sum / l_line_count;
      l_y2 = l_y2_sum / l_line_count;

      originLeft_x = (480-l_y1)*(l_x1-l_x2)/(l_y1-l_y2)+l_x1;

      l_descent = (float)(l_x1-l_x2)/(float)(l_y1-l_y2);
      l_descent = fabsf(l_descent);
      l_descent *= 160;

      if(l_descent<200) l_descent=0;

      left_x = originLeft_x + l_descent;
      line(image, Point(l_x1,l_y1), Point(l_x2,l_y2), Scalar(0, 255, 255), 6);
  }

  if(r_line_count>10){
      rightLine_ok=1;
      r_x1 = r_x1_sum / r_line_count;
      r_x2 = r_x2_sum / r_line_count;
      r_y1 = r_y1_sum / r_line_count;
      r_y2 = r_y2_sum / r_line_count;

      originRight_x = (480-r_y1)*(r_x1-r_x2)/(r_y1-r_y2)+r_x1;

      r_descent = (float)(r_x1-r_x2)/(float)(r_y1-r_y2);
      r_descent = -fabsf(r_descent);
      r_descent *= 160;

      if(r_descent>-200) r_descent=0;

      right_x = originRight_x + r_descent;
      line(image, Point(r_x1,r_y1), Point(r_x2,r_y2), Scalar(0, 255, 255), 6);

  }

  // if(left_x < -350) left_x=-350;
  // if(left_x > 350) left_x=350;
  // if(right_x > 990) right_x=990;
  // if(right_x < 290) right_x=290;
   if(left_x < -500) left_x=-500;
   if(left_x > 500) left_x=500;
   if(right_x > 1140) right_x=1140;
   if(right_x < 140) right_x=140;

  center_x = left_x + right_x;

  if(disp_center_x){

    cout << "originLeft_x: " << originLeft_x << endl;
    cout << "originRight_x: " << originRight_x << endl;
    cout << "l_descent: " << l_descent << endl;
    cout << "r_descent: " << r_descent << endl;
    cout << "left_x: " << left_x << endl;
    cout << "right_x: " << right_x << endl;
    cout << "center: " << center_x << endl;
    cout << "error: " << 640-center_x << endl;
  }

  l_descent = 0;
  r_descent = 0;

  /****************publish******************/

  std_msgs::Int32MultiArray imshow_msg;
  imshow_msg.data.clear();

  int sendData[8] = {l_x1,l_x2,l_y1,l_y2,r_x1,r_x2,r_y1,r_y2};
  for (int i = 0; i < 8; i++)
	{
		imshow_msg.data.push_back(sendData[i]);
	}
  imshow_pub.publish(imshow_msg);

  /*------------------------------------------*/

  sherlotics::LINEtoPID send_msg;
  if(leftLine_ok==1 && rightLine_ok==1){ // 양쪽
    send_msg.data = 640-center_x;
    // cout << "---------------both!!!!!!!!------------" << endl;
    // cout << "error: " << 640-center_x << endl;
  }
  else if(leftLine_ok==1 && rightLine_ok==0){ // 왼쪽만
    send_msg.data = -left_x;
    // cout << "---------------Left-----------------" << endl;
    // cout << "error: " << -left_x << endl;

  }
  else if(leftLine_ok==0 && rightLine_ok==1){ // 오른쪽만
    send_msg.data = 640-right_x;
    // cout << "---------------Right-----------------" << endl;
    // cout << "error: " << 640-right_x << endl;
  }
  else if(leftLine_ok==0 && rightLine_ok==0){ // 없음
    send_msg.data = 0;
    // cout << "-----------****nothing****-------------" << endl;
    // cout << "error: 0" << endl;
  }

  pub.publish(send_msg);// + left - right

  waitKey(WAITKEYSIZE);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "linePart_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("normalMode_msg", QUEUESIZE, imageCallback);

  pub = nh.advertise<sherlotics::LINEtoPID>("LINEtoPID_msg", QUEUESIZE);
  imshow_pub = nh.advertise<std_msgs::Int32MultiArray>("LINEtoIMSHOW_msg", QUEUESIZE);

  ros::spin();

  return 0;
}
