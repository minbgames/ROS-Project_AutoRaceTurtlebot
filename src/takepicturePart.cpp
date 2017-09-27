#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sherlotics/variable.hpp>
#include <string>

using namespace cv;
using namespace std;

/********************************CHANGE******************************/

int ShowBin_pic = 0;

Scalar lowerBlue_tr(101, 100, 82);
Scalar upperBlue_tr(114, 190, 129);
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
Mat takePicture;

int key_input;
char path[33]="src/sherlotics/scripts/img_data/";
char jpg[5]=".jpg";
char underbar[2]="_";
char num_i[3], num_j[3];
int i_num=0,j_num=0;

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

  resize( image_raw, image_raw, Size( WIDTH_SIZE, HEIGHT_SIZE ), 0, 0, CV_INTER_CUBIC );

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

  //morphological opening 작은 점들을 제거
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

  if(ShowBin_pic){
    imshow("PictureBin1_pic",binaryImage_1);
    imshow("PictureBin2_pic",binaryImage_2);
  }

  //라벨링
  Mat img_labels_1,stats_1, centroids_1;
  int numOfLables_1 = connectedComponentsWithStats(binaryImage_1, img_labels_1,
                                             stats_1, centroids_1, 8,CV_32S);

  Mat img_labels_2,stats_2, centroids_2;
  int numOfLables_2 = connectedComponentsWithStats(binaryImage_2, img_labels_2,
                                             stats_2, centroids_2, 8,CV_32S);

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
  Rect region_rect;
  if(max_1 >= max_2){
    int left = stats_1.at<int>(idx_1, CC_STAT_LEFT);
    int top  = stats_1.at<int>(idx_1, CC_STAT_TOP);
    int width = stats_1.at<int>(idx_1, CC_STAT_WIDTH);
    int height  = stats_1.at<int>(idx_1, CC_STAT_HEIGHT);

    Rect region_rect_1(Point(leftRect_tr_1+left,topRect_tr_1+top), Point(leftRect_tr_1+left+width,topRect_tr_1+top+height));
    region_rect=region_rect_1;
  }
  else{
    int left = stats_2.at<int>(idx_2, CC_STAT_LEFT);
    int top  = stats_2.at<int>(idx_2, CC_STAT_TOP);
    int width = stats_2.at<int>(idx_2, CC_STAT_WIDTH);
    int height  = stats_2.at<int>(idx_2, CC_STAT_HEIGHT);

    Rect region_rect_2(Point(leftRect_tr_2+left,topRect_tr_2+top), Point(leftRect_tr_2+left+width,topRect_tr_2+top+height));
    region_rect=region_rect_2;
  }

  rectangle( image_raw, region_rect,Scalar(0,0,255),1 );

  imshow("binary_1", binaryImage_1);
  imshow("binary_2", binaryImage_2);
  imshow("original", image_raw);

  key_input = waitKey(WAITKEYSIZE);

  if (key_input == 32){ // SPACE BAR

    takePicture = image_raw(region_rect);
    resize( takePicture, takePicture, Size( 50, 50 ), 0, 0, CV_INTER_CUBIC );
    imshow("takePicture",takePicture);

    i_num = 0;
    char str[50] = "";
    sprintf(num_i, "%d", i_num+1);
    sprintf(num_j, "%d", j_num+1);
    strcat(str, path);
    strcat(str, num_i);
    strcat(str, underbar);
    strcat(str, num_j);
    strcat(str, jpg);
    imwrite(str,takePicture);
    j_num++;
    if(j_num==10){
      j_num=0;
      i_num++;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "takepicturePart_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image_raw", QUEUESIZE, imageCallback);

  ros::spin();

  return 0;
}
