#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sherlotics/variable.hpp>

#define RESIZED_SIZE 50

using namespace cv;

/********************************CHANGE******************************/

int ShowBin_tr = 0;

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

/********************************CHANGE******************************/

Mat image_raw;
Mat roi_tr_1;
Mat roi_tr_2;
Mat hsvImage_1;
Mat hsvImage_2;
Mat binaryImageRed1_1;
Mat binaryImageRed2_1;
Mat binaryImageBlue_1;
Mat binaryImageRedMerge_1;
Mat binaryImage_1;
Mat binaryImageRed1_2;
Mat binaryImageRed2_2;
Mat binaryImageBlue_2;
Mat binaryImageRedMerge_2;
Mat binaryImage_2;

Mat traffic_to_learning_image;
Mat wantedImage;
Mat resizedImage;
Mat roi_tr;

float ratio=0;
int traffic_ok=0;

image_transport::Publisher learningSignPub;

ros::Publisher imshow_pub;

void trafficImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  traffic_ok=0;
  try
  {
    image_raw = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  roi_tr_1 =image_raw(Rect(leftRect_tr_1,topRect_tr_1,rightRect_tr_1-leftRect_tr_1,bottomRect_tr_1-topRect_tr_1));
  roi_tr_2 =image_raw(Rect(leftRect_tr_2,topRect_tr_2,rightRect_tr_2-leftRect_tr_2,bottomRect_tr_2-topRect_tr_2));

  cvtColor(roi_tr_1, hsvImage_1, COLOR_BGR2HSV);
  cvtColor(roi_tr_2, hsvImage_2, COLOR_BGR2HSV);

  inRange(hsvImage_1, lowerRed1_tr, upperRed1_tr, binaryImageRed1_1);
  inRange(hsvImage_1, lowerRed2_tr, upperRed2_tr, binaryImageRed2_1);
  inRange(hsvImage_1, lowerBlue_tr, upperBlue_tr, binaryImageBlue_1);
  inRange(hsvImage_2, lowerRed1_tr, upperRed1_tr, binaryImageRed1_2);
  inRange(hsvImage_2, lowerRed2_tr, upperRed2_tr, binaryImageRed2_2);
  inRange(hsvImage_2, lowerBlue_tr, upperBlue_tr, binaryImageBlue_2);

  binaryImageRedMerge_1 = binaryImageRed1_1 | binaryImageRed2_1;
  binaryImage_1 = binaryImageRedMerge_1 | binaryImageBlue_1;

  binaryImageRedMerge_2 = binaryImageRed1_2 | binaryImageRed2_2;
  binaryImage_2 = binaryImageRedMerge_2 | binaryImageBlue_2;

  erode(binaryImage_1, binaryImage_1, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( binaryImage_1, binaryImage_1, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  //morphological closing 영역의 구멍 메우기
  dilate( binaryImage_1, binaryImage_1, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(binaryImage_1, binaryImage_1, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  //morphological opening 작은 점들을 제거
  erode(binaryImage_2, binaryImage_2, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  dilate( binaryImage_2, binaryImage_2, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  //morphological closing 영역의 구멍 메우기
  dilate( binaryImage_2, binaryImage_2, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );
  erode(binaryImage_2, binaryImage_2, getStructuringElement(MORPH_ELLIPSE, Size(MORSIZE, MORSIZE)) );

  Mat img_labels_1,stats_1, centroids_1;
  int numOfLables_1 = connectedComponentsWithStats(binaryImage_1, img_labels_1,
                                             stats_1, centroids_1, 8,CV_32S);

  Mat img_labels_2,stats_2, centroids_2;
  int numOfLables_2 = connectedComponentsWithStats(binaryImage_2, img_labels_2,
                                             stats_2, centroids_2, 8,CV_32S);

  if(ShowBin_tr){
    imshow("Left_traffic_tr",binaryImage_1);
    imshow("Right_traffic_tr",binaryImage_2);
  }

  //영역박스 그리기
  int max_1 = -1, idx_1=0;
  for (int j = 1; j < numOfLables_1; j++) {
      int area_1 = stats_1.at<int>(j, CC_STAT_AREA);
      if ( max_1 < area_1 )
      {
          max_1 = area_1;
          idx_1 = j;
      }
  }

  //영역박스 그리기
  int max_2 = -1, idx_2=0;
  for (int j = 1; j < numOfLables_2; j++) {
      int area_2 = stats_2.at<int>(j, CC_STAT_AREA);
      if ( max_2 < area_2 )
      {
          max_2 = area_2;
          idx_2 = j;
      }
  }

  Rect rect;
  int left,top,width,height;
  int leftRect_tr,topRect_tr;
  if(max_1 >= max_2){
    left = stats_1.at<int>(idx_1, CC_STAT_LEFT);
    top  = stats_1.at<int>(idx_1, CC_STAT_TOP);
    width = stats_1.at<int>(idx_1, CC_STAT_WIDTH);
    height  = stats_1.at<int>(idx_1, CC_STAT_HEIGHT);

    leftRect_tr=leftRect_tr_1;
    topRect_tr=topRect_tr_1;
  }
  else{
    left = stats_2.at<int>(idx_2, CC_STAT_LEFT);
    top  = stats_2.at<int>(idx_2, CC_STAT_TOP);
    width = stats_2.at<int>(idx_2, CC_STAT_WIDTH);
    height  = stats_2.at<int>(idx_2, CC_STAT_HEIGHT);

    leftRect_tr=leftRect_tr_2;
    topRect_tr=topRect_tr_2;
  }

  ratio = (float)width/height;

  //---------------------condition-------------------------

  if(height>TRAFFICHEIGHT_MIN && height<TRAFFICHEIGHT_MAX ){
    if(width>TRAFFICWIDTH_MIN && width<TRAFFICWIDTH_MAX ){
      if(ratio>TRAFFICRATIO_MIN && ratio<TRAFFICRATIO_MAX ){
        traffic_ok=1;
      }
    }
  }

  if(traffic_ok){
    Rect rect(Point(leftRect_tr+left,topRect_tr+top), Point(leftRect_tr+left+width,topRect_tr+top+height));

    wantedImage = image_raw(rect);

    resize( wantedImage, traffic_to_learning_image, Size( RESIZED_SIZE, RESIZED_SIZE ), 0, 0, CV_INTER_CUBIC );
    //cvtColor(resizedImage, traffic_to_learning_image, CV_BGR2GRAY);

    //imshow("traffic_to_learning_image",traffic_to_learning_image);

    sensor_msgs::ImagePtr learningSignMsg;
    learningSignMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", traffic_to_learning_image).toImageMsg();
    learningSignPub.publish(learningSignMsg);
  }
  else{
    left=0, top=0, width=0, height=0;
  }

  std_msgs::Int32MultiArray imshow_msg;
  imshow_msg.data.clear();

  int sendData[4] = {left+leftRect_tr,top+topRect_tr,width,height};

  for (int i = 0; i < 4; i++)
	{
		imshow_msg.data.push_back(sendData[i]);
	}
  imshow_pub.publish(imshow_msg);

  waitKey(WAITKEYSIZE);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trafficPart_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("normalMode_msg", QUEUESIZE, trafficImageCallback);

  learningSignPub = it.advertise("TRAFFICtoLEARNING_msg", QUEUESIZE);
  imshow_pub = nh.advertise<std_msgs::Int32MultiArray>("TRAFFICtoIMSHOW_msg", QUEUESIZE);

  ros::spin();

  return 0;
}
