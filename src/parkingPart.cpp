#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sherlotics/PARKINGtoCENTER.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sherlotics/variable.hpp>
#include <cstdio>

//우측 차선(점선)검출용 ROI
#define ROI1_X 320
#define ROI1_Y 100
#define ROI1_WIDTH 320
#define ROI1_HEIGHT 380

//주차 시 가로선 검출 ROI
#define ROI2_X 0
#define ROI2_Y 100
#define ROI2_WIDTH 640
#define ROI2_HEIGHT 380

//시간조정시 90도 회전시간을 기준으로
//비례하여 나머지 값들을 일괄조정
//90도 회전시간
//#define ROTATION_VAL 52 148
#define ROTATION_VAL 74
//주차칸 인식시간
//#define PARK1_VAL 6 17
#define PARK1_VAL 8
//#define PARK2_VAL 8 23
#define PARK2_VAL 11
//주차칸 검사시간
//#define DETER_VAL 8 23
#define DETER_VAL 11
//주차칸 진입시간
//#define IN_VAL 4 11
#define IN_VAL 5
//주차칸내 대기시간
//#define WAIT_VAL 30 30
#define WAIT_VAL 15
//주차칸 진출시간
//#define OUT_VAL 8 23
#define OUT_VAL 11


using namespace cv;
using namespace std;

ros::Publisher pub;
ros::Publisher mode_pub;

//입력 프레임
Mat frame;

//ROI1 그레이
Mat gray1;
//ROI1 모폴로지
Mat binOpen1;
Mat binClose1;
//ROI 경계선추출
Mat canny;

//ROI2그레이, 모폴로지
Mat gray2;
Mat binOpen2;

//ROI1 레이블링
Mat labels1, stats1, centroids1;
int numLabels1;

//ROI2 레이블링
Mat labels2, stats2, centroids2;
int numLabels2;

//상태 (아래 switch문 참고)
int state = 0;
//상태 지속시간
int stateCnt = 0;
//동작번호(actToOutput()함수 참조)
int actNo = 0;

//for debugging
int testvar = 0;

//엣지추출
std::vector<Vec4i> lines;

void actToOutput(int act, float *output);

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

  cout << "Parking Part on??" << endl;
  /**************subscribe******************/

  //if(0){
  //  robotMode = PARKING_MODE;
  //}
  //PARKING_MODE
  //NORMAL_TO_PARKING
  //NORMAL_MODE



  /****************process******************/
  //점선 검출용 ROI와, 주차시 가로선 검출 ROI를 지정
  Rect roi_rect1 = Rect(ROI1_X, ROI1_Y, ROI1_WIDTH, ROI1_HEIGHT);
  Mat roi1 = frame(roi_rect1);
  cvtColor(roi1, gray1, CV_BGR2GRAY);


  Rect roi_rect2 = Rect(ROI2_X, ROI2_Y, ROI2_WIDTH, ROI2_HEIGHT);
  Mat roi2 = frame(roi_rect2);
  cvtColor(roi2, gray2, CV_BGR2GRAY);

  /******************* minbaek *************************/

  //adaptiveThreshold(gray, bin, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 37, 5);
  threshold(gray1, binOpen1, 190, 255, THRESH_BINARY);
  threshold(gray1, binClose1, 190, 255, THRESH_BINARY);

  threshold(gray2, binOpen2, 190, 255, THRESH_BINARY);
  //threshold(gray, bin, 84, 255, THRESH_BINARY);

  /******************* minbaek *************************/

  //모폴로지 침식, 팽창연산
  Mat element5(5, 5, CV_8U, Scalar(1));
  morphologyEx(binOpen1, binOpen1,  MORPH_OPEN, element5);
  morphologyEx(binClose1, binClose1,  MORPH_CLOSE, element5);

  morphologyEx(binOpen2, binOpen2,  MORPH_OPEN, element5);
  //케니엣지 추출
  Canny(binClose1, canny, 125, 250);


  //레이블링 알고리즘
  numLabels1 = connectedComponentsWithStats(binOpen1, labels1, stats1, centroids1, 8, CV_32S);
  numLabels2 = connectedComponentsWithStats(binOpen2, labels2, stats2, centroids2, 8, CV_32S);
  //HoughLinesP(canny, lines, 1, CV_PI/180, 40, 20, 20);
  //허프변환
  HoughLinesP(canny, lines, 1, CV_PI/180, 30, 20, 20);


  //점선 개수 인식
  int numRect = 0;
  int strictNumRect = 0;
  for(int i=1; i<numLabels1; i++) {
    int area = stats1.at<int>(i, CC_STAT_AREA);
    int left = stats1.at<int>(i, CC_STAT_LEFT);
    int top = stats1.at<int>(i, CC_STAT_TOP);
    int width = stats1.at<int>(i, CC_STAT_WIDTH);
    int height = stats1.at<int>(i, CC_STAT_HEIGHT);

    if(width<190 && height<60 && top>40) {
      ++numRect;
      if(area > 400) {
        ++strictNumRect;
      }
    }
  }

  //imshow("park", binOpen1);

  //가로선 크기, 위치인식
  int numStop = 0;
  int numStopY = 0;
  for(int i=1; i<numLabels2; i++) {
    int area = stats2.at<int>(i, CC_STAT_AREA);
    int left = stats2.at<int>(i, CC_STAT_LEFT);
    int top = stats2.at<int>(i, CC_STAT_TOP);
    int width = stats2.at<int>(i, CC_STAT_WIDTH);
    int height = stats2.at<int>(i, CC_STAT_HEIGHT);


    if(width>600 && height<200 && (top+height/2)>120) {
      ++numStop;
      numStopY = top+height/2;
    }
  }

  //상태전이문
  switch(state) {
    //첫번째 주차칸 인식
    case 0:
      actNo = 1;
      //actNo = 3;
      robotMode = NORMAL_TO_PARKING;
      //robotMode = PARKING_MODE;
      if(numRect == 7 || numRect == 8) {
        ++stateCnt;
      }
      else if(numRect != 6 && numRect != 7) {
        stateCnt = 0;
      }
      if(stateCnt >= PARK1_VAL) {
        stateCnt = 0;
        state = 1;
      }

      break;
    //첫번째 주차칸 앞 회전
    case 1:
      actNo = 4;
      robotMode = PARKING_MODE;
      ++stateCnt;
      if(stateCnt > ROTATION_VAL) {
        stateCnt = 0;
        state = 2;
      }
      break;
    //첫번째 주차칸 검사
    case 2:
      actNo = 0;
      robotMode = PARKING_MODE;
      if(numStop > 0) {
        ++stateCnt;
      }
      else {
        --stateCnt;
      }
      if(stateCnt >= DETER_VAL) {
        stateCnt = 0;
        state = 3;
      }
      else if(stateCnt <= -1*DETER_VAL) {
        stateCnt = 0;
        state = 8;
      }
      break;
    //주차칸 진입 (첫번째, 두번째 주차칸 공통)
    case 3:
      actNo = 1;
      robotMode = PARKING_MODE;
      if(numStop == 0) {
        ++stateCnt;
      }
      else {
        stateCnt = 0;
      }
      if(stateCnt >= IN_VAL) {
        stateCnt = 0;
        state = 4;
      }
      break;
    //주차칸 내 대기(첫번째, 두번째 주차칸 공통)
    case 4:
      actNo = 0;
      robotMode = PARKING_MODE;
      ++stateCnt;
      if(stateCnt >= WAIT_VAL) {
        stateCnt = 0;
        state = 5;
      }
      break;
    //주차칸 진출(첫번째, 두번째 주차칸 공통)
    case 5:
      actNo = 2;
      robotMode = PARKING_MODE;
      if(numStop == 1 && numStopY>180 && numStopY<250) {
        ++stateCnt;
      }
      else {
        stateCnt = 0;
      }
      if(stateCnt >= OUT_VAL) {
        stateCnt = 0;
        state = 6;
      }
      break;
    //회전하여 차선복귀(첫번째, 두번째 주차칸 공통)
    case 6:
      actNo = 3;
      robotMode = PARKING_MODE;
      ++stateCnt;
      if(stateCnt > ROTATION_VAL) {
        stateCnt = 0;
        state = 7;
      }
      break;
    //노말모드 전환 (주차동작 종료후 다음파트로)
    case 7:
      actNo = 1;
      robotMode = NORMAL_MODE;
      break;
    //90도 회전하여 차선방향 보기
    case 8:
      actNo = 3;
      robotMode = PARKING_MODE;
      ++stateCnt;
      if(stateCnt > ROTATION_VAL) {
        stateCnt = 0;
        state = 9;
      }
      break;
    //두번째 주차칸 인식
    case 9:
      actNo = 1;
      robotMode = NORMAL_TO_PARKING;
      if(strictNumRect == 0) {
        ++stateCnt;
      }
      else if(strictNumRect != 1){
        stateCnt = 0;
      }
      if(stateCnt >= PARK2_VAL) {
        stateCnt = 0;
        state = 10;
      }
      break;
    //두번째 주차칸 앞 회전
    case 10:
      actNo = 4;
      robotMode = PARKING_MODE;
      ++stateCnt;
      if(stateCnt > ROTATION_VAL) {
        stateCnt = 0;
        state = 3;
      }
      break;
    //디폴트(사용되지 않음)
    default:
      robotMode = PARKING_MODE;
      actNo = 0;
  }


  printf("////////// [ %d %d %d]\n", state, numRect, strictNumRect);


  for(int i=0; i<lines.size(); i++) {
    Vec4i lv = lines[i];
    float dist = norm(Mat(Point(lv[0], lv[1])), Mat(Point(lv[2], lv[3])));
    int near = 0;
    for(int j=1; j<numLabels1; j++) {
      int left = stats1.at<int>(j, CC_STAT_LEFT);
      int top = stats1.at<int>(j, CC_STAT_TOP);
      int width = stats1.at<int>(j, CC_STAT_WIDTH);
      int height = stats1.at<int>(j, CC_STAT_HEIGHT);
      int px = left + width/2;
      int py = top + height/2;
      float rho = (float)((px-lv[0])*(lv[3]-lv[1]) - (py-lv[1])*(lv[4]-lv[2]))/dist;
      if(rho<400 && rho>-400) {
        near += 1;
      }
    }
  }




  float sendData[2];
  actToOutput(actNo, sendData);

  /****************publish******************/
  sherlotics::PARKINGtoCENTER mode_msg;
  mode_msg.data = robotMode;
  mode_pub.publish(mode_msg);

  if(robotMode == PARKING_MODE){
    std_msgs::Float32MultiArray PARKINGtoTELEOP_msg;
    PARKINGtoTELEOP_msg.data.clear();

    for (int i = 0; i < 2; i++)
      {
          PARKINGtoTELEOP_msg.data.push_back(sendData[i]);
      }
    pub.publish(PARKINGtoTELEOP_msg);
  }

  waitKey(WAITKEYSIZE);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "parkingPart_node");

  std::ifstream file2("/home/m/initmode.txt");
  file2 >> robotMode;
  file2.close();

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("parkingMode_msg", QUEUESIZE, imageCallback);

  pub = nh.advertise<std_msgs::Float32MultiArray>("TELEOP_msg", QUEUESIZE);
  mode_pub = nh.advertise<sherlotics::PARKINGtoCENTER>("PARKINGtoCENTER_msg", QUEUESIZE);

  ros::spin();

  return 0;
}


//동작정의함수
//output[0]은 직진속도
//output[1]은 회전속도
void actToOutput(int act, float *output)
{
  switch(act) {
    case 0:
      output[0] = 0.0f;
      output[1] = 0.0f;
      break;
    case 1:
      output[0] = 0.06f;
      output[1] = 0.0f;
      break;
    case 2:
      output[0] = -0.06f;
      output[1] = 0.0f;
      break;
    case 3:
      output[0] = 0.0f;
      output[1] = 0.2f;
      break;
    case 4:
      output[0] = 0.0f;
      output[1] = -0.2f;
      break;
  }
}
