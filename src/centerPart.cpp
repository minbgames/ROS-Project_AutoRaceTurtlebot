#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sherlotics/PARKINGtoCENTER.h>
#include <sherlotics/LIDARtoCENTER.h>
#include <sherlotics/LEARNINGtoCENTER.h>
#include <sherlotics/CENTERtoPID.h>
#include <sherlotics/CENTERtoLIDAR.h>
#include <sherlotics/CENTERtoTRAFFIC.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sherlotics/variable.hpp>
#include <iostream>

using namespace std;
using namespace cv;

Mat image_raw;

ros::Publisher mode_pub;
ros::Publisher mode_lidar_pub;
ros::Publisher mode_traffic_pub;
image_transport::Publisher normal_pub;
image_transport::Publisher parking_pub;
image_transport::Publisher imshow_pub;

int fontFace = 1;
double fontScale = 4;

void msgCallback2(const sherlotics::PARKINGtoCENTER::ConstPtr& msg )
{
  if(robotMode==PARKING_MODE || robotMode==NORMAL_TO_PARKING){
    robotMode=msg->data;
  }
}

void msgCallback3(const sherlotics::LIDARtoCENTER::ConstPtr& msg )
{
  if(robotMode==LIDAR_MODE){
    robotMode=msg->data;
  }
}

void msgCallback4(const sherlotics::LEARNINGtoCENTER::ConstPtr& msg )
{
  if(robotMode==NORMAL_MODE){
    robotMode=msg->data;
  }
}

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

  // cout << "robotMode: " << robotMode << endl;
  if(robotMode==NORMAL_MODE){
    sensor_msgs::ImagePtr normalMode_msg;
    normalMode_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_raw).toImageMsg();
    normal_pub.publish(normalMode_msg);

    string normalText = "Normal Mode";
    putText( image_raw, normalText, Point(120,260), fontFace, fontScale, Scalar(0,0,255), 8 );
  }

  if(robotMode==PARKING_MODE){
    sensor_msgs::ImagePtr parkingMode_msg;
    parkingMode_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_raw).toImageMsg();
    parking_pub.publish(parkingMode_msg);

    string prakingText = "Parking Mode";
    putText( image_raw, prakingText, Point(120,260), fontFace, fontScale, Scalar(0,0,255), 8 );
  }

  if(robotMode==NORMAL_TO_PARKING){
    sensor_msgs::ImagePtr normalMode_msg;
    normalMode_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_raw).toImageMsg();
    normal_pub.publish(normalMode_msg);

    sensor_msgs::ImagePtr parkingMode_msg;
    parkingMode_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_raw).toImageMsg();
    parking_pub.publish(parkingMode_msg);

    string prakingText = "Normal To Parking";
    putText( image_raw, prakingText, Point(20,260), fontFace, fontScale, Scalar(0,0,255), 8 );
  }

  if(robotMode==LIDAR_MODE){
    string lidarText = "Lidar Mode";
    putText( image_raw, lidarText, Point(120,260), fontFace, fontScale, Scalar(0,0,255), 8 );
  }

  sensor_msgs::ImagePtr imshow_msg;
  imshow_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_raw).toImageMsg();
  imshow_pub.publish(imshow_msg);

  sherlotics::CENTERtoPID mode_msg;
  mode_msg.data = robotMode;
  mode_pub.publish(mode_msg);

  sherlotics::CENTERtoLIDAR mode_lidar_msg;
  mode_lidar_msg.data = robotMode;
  mode_lidar_pub.publish(mode_lidar_msg);

  sherlotics::CENTERtoTRAFFIC mode_traffic_msg;
  mode_traffic_msg.data = robotMode;
  mode_traffic_pub.publish(mode_traffic_msg);

  waitKey(WAITKEYSIZE);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "centerPart_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image_raw", QUEUESIZE, imageCallback);
  ros::Subscriber sub2 = nh.subscribe("PARKINGtoCENTER_msg", QUEUESIZE, msgCallback2);
  ros::Subscriber sub3 = nh.subscribe("LIDARtoCENTER_msg", QUEUESIZE, msgCallback3);
  ros::Subscriber sub4 = nh.subscribe("LEARNINGtoCENTER_msg", QUEUESIZE, msgCallback4);

  imshow_pub = it.advertise("imshow_msg", QUEUESIZE);
  normal_pub = it.advertise("normalMode_msg", QUEUESIZE);
  parking_pub = it.advertise("parkingMode_msg", QUEUESIZE);
  mode_pub = nh.advertise<sherlotics::CENTERtoPID>("CENTERtoPID_msg", QUEUESIZE);
  mode_lidar_pub = nh.advertise<sherlotics::CENTERtoLIDAR>("CENTERtoLIDAR_msg", QUEUESIZE);
  mode_traffic_pub = nh.advertise<sherlotics::CENTERtoTRAFFIC>("CENTERtoTRAFFIC_msg", QUEUESIZE);

  ros::spin();

  return 0;
}
