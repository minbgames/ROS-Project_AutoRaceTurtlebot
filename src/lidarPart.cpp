#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/UInt16.h>
#include <nav_msgs/Odometry.h>
#include <sherlotics/LIDARtoCENTER.h>
#include <sherlotics/CENTERtoLIDAR.h>
#include <sherlotics/LINEtoPID.h>
#include <sherlotics/variable.hpp>
#include <iostream>
#include <cmath>

#define LIDAR_RANGE 80
#define ANGLEHAT_GAP M_PI/2.0
#define AVOID_GAP 30

#define LENGTH_MIN 100
#define AVOID_LENGTH_MAX 60

using namespace std;
using namespace cv;

int start_ok=1;
double init_x=0, x_m, x_p, x_hat, x_distance;
double init_y=0, y_m, y_p, y_hat, y_distance;
double distance_error;
double final_x, final_y;
double init_angle=0, o_x, o_y, o_z, o_w, angle_p, angle_hat;
double object_angle;

float length, angle, final_angle, avoid_angle;
int x, y;
int final_dot, avoid_final_dot;

double middle_angle;

double object_x=2.5;
double object_y=2.0;

#define DISTANCE_ERROR_MAX 2.7
#define AVOID_NUMBER_MAX 5

ros::Publisher pub;
ros::Publisher mode_pub;
image_transport::Publisher lidar_pub;

void modeCallback(const sherlotics::CENTERtoLIDAR::ConstPtr& msg )
{
  robotMode=msg->data;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg ) //1 and 0
{
  x_p = msg -> pose.pose.position.x;
  y_p = msg -> pose.pose.position.y;

  o_x = msg -> pose.pose.orientation.x;
  o_y = msg -> pose.pose.orientation.y;
  o_z = msg -> pose.pose.orientation.z;
  o_w = msg -> pose.pose.orientation.w;

  angle_p=atan2(2*(o_y*o_x+o_w*o_z),o_w*o_w+o_x*o_x-o_y*o_y-o_z*o_z);

  if(robotMode==LIDAR_MODE && start_ok==1){
    init_x=x_p;
    init_y=y_p;
    init_angle = angle_p;
    start_ok=0;
  }

  x_m = x_p-init_x;
  y_m = y_p-init_y;

  angle_hat = angle_p - init_angle;

  if(angle_hat>M_PI) angle_hat=angle_hat-2*M_PI;
  if(angle_hat<-M_PI) angle_hat=angle_hat+2*M_PI;

  x_hat = cos(init_angle)*x_m + sin(init_angle)*y_m;
  y_hat = -sin(init_angle)*x_m + cos(init_angle)*y_m;
  /* rotation matrix  [ x_hat   =  [ c s   [ x_m
                        y_hat ]     -s c ]   y_m ]  */

  if(disp_pose){
    cout << "origin pose.x: " << x_p << endl;
    cout << "origin pose.y: " << y_p << endl;
    cout << "origin angle: " << angle_p << endl;
    cout << "x_hat: " << x_hat << endl;
    cout << "y_hat: " << y_hat << endl;
    cout << "angle_hat: " << angle_hat << endl; // -PI ~ PI
  }

}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg ) //1 and 0
{
  // cout << "angle_min  " << msg -> angle_min<< endl;
  // cout << "angle_max  " << msg -> angle_max<< endl;
  // cout << "angle_increment  " << msg -> angle_increment<< endl;
  // cout << "scan_time  " << msg -> scan_time<< endl;
  // cout << "range_min  " << msg -> range_min<< endl;
  // cout << "range_max  " << msg -> range_max<< endl;
  // cout << sizeof(msg -> ranges) << endl;

  float extract_dot[360];
  float avoid_dot[360];
  int dot_number=0;
  int avoid_number=0;
  Mat plotImage(800,800,CV_8UC3,Scalar(0,0,0));
  double plot_anglehat, anglehat_gap, avoid_gap1, avoid_gap2, lidar_angle;

  if(angle_hat<0) plot_anglehat = 2*M_PI + angle_hat;
  else plot_anglehat = angle_hat;
  //plot_anglehat 0 ~ 2PI

  x_distance = object_x-x_hat;
  y_distance = object_y-y_hat;


  distance_error = (float)(sqrt(pow(x_distance,2)+pow(y_distance,2)));


  cout << "x_hat: " << x_hat << endl;
  cout << "y_hat: " << y_hat << endl;
  cout << "x_distance:" << x_distance <<endl;
  cout << "y_distance:" << y_distance <<endl;
  cout << "distance_error:" << distance_error <<endl;

  if(x_distance<0.01){
    if(y_distance<0.01){
        robotMode=NORMAL_MODE;
    }
  }

  if(x_distance == 0){
    if(y_distance>=0) object_angle = M_PI/2.0;
    else object_angle = -M_PI/2.0;
  }
  else if(y_distance == 0){
    if(x_distance>=0) object_angle = 0;
    else object_angle = -M_PI;
  }
  else if(x_distance < 0 && y_distance < 0){
    object_angle=M_PI+atan(y_distance/x_distance);
  }
  else if(x_distance < 0){
    object_angle=M_PI-atan(y_distance/-x_distance);
  }
  else if(y_distance < 0){
    object_angle=2*M_PI-atan(-y_distance/x_distance);
  }
  else{
    object_angle=2*M_PI+atan(y_distance/x_distance);
  }

  if (object_angle<0) object_angle+=2.0*M_PI;
  //object_angle 0 ~ 2PI

  for (int i = 360-LIDAR_RANGE; i < 360; i++) {
    length = (float)(150*(msg->ranges[i]));
    angle = (float)i * M_PI / 180.0;

    lidar_angle = plot_anglehat+angle-2.0*M_PI;
    if(lidar_angle<0) lidar_angle+=2.0*M_PI;

    anglehat_gap = fabs(lidar_angle-object_angle);
    if(anglehat_gap>M_PI) anglehat_gap = 2.0*M_PI - anglehat_gap;

    if(anglehat_gap<ANGLEHAT_GAP){
      if((length > LENGTH_MIN) || (length == 0)){

        extract_dot[dot_number]=lidar_angle *180.0 / M_PI;
        dot_number++;

        if(length==0){
          x=(int)(400*cos(lidar_angle));
          y=-(int)(400*sin(lidar_angle));
        }
        else{
          x=(int)(length*cos(lidar_angle));
          y=-(int)(length*sin(lidar_angle));
        }

        line(plotImage, Point(400,400), Point(400+x,400+y), Scalar(0,150,0), 1);
      }
      else if(length < AVOID_LENGTH_MAX){

        avoid_dot[avoid_number]=lidar_angle *180.0 / M_PI;
        avoid_number++;

        x=(int)(length*cos(lidar_angle));
        y=-(int)(length*sin(lidar_angle));

        line(plotImage, Point(400,400), Point(400+x,400+y), Scalar(0,0,150), 1);
      }
      else{
        line(plotImage, Point(400,400), Point(400+x,400+y), Scalar(255,255,255), 1);
      }
    }
  }

  for (int i = 0; i < LIDAR_RANGE; i++) {
    length = (float)(150*(msg->ranges[i]));
    angle = (float)i * M_PI / 180.0;

    lidar_angle = plot_anglehat+angle;
    if(lidar_angle<0) lidar_angle+=2.0*M_PI;  //0 ~2PI

    anglehat_gap = fabs(lidar_angle-object_angle);
    if(anglehat_gap>M_PI) anglehat_gap = 2.0*M_PI - anglehat_gap;

    if(anglehat_gap<ANGLEHAT_GAP){
      if((length > LENGTH_MIN) || (length == 0)){
        extract_dot[dot_number]=lidar_angle*180/M_PI;
        dot_number++;

        if(length==0){
          x=(int)(400*cos(lidar_angle));
          y=-(int)(400*sin(lidar_angle));
        }
        else{
          x=(int)(length*cos(lidar_angle));
          y=-(int)(length*sin(lidar_angle));
        }

        line(plotImage, Point(400,400), Point(400+x,400+y), Scalar(0,150,0), 1);
      }
      else if(length < AVOID_LENGTH_MAX){

        avoid_dot[avoid_number]=lidar_angle *180.0 / M_PI;
        avoid_number++;

        x=(int)(length*cos(lidar_angle));
        y=-(int)(length*sin(lidar_angle));

        line(plotImage, Point(400,400), Point(400+x,400+y), Scalar(0,0,150), 1);
      }
      else{
        x=(int)(length*cos(lidar_angle));
        y=-(int)(length*sin(lidar_angle));

        line(plotImage, Point(400,400), Point(400+x,400+y), Scalar(255,255,255), 1);
      }
    }
  }

  /**************** 라이다 추출!!! *********************/

  float avoid_target_dot=0, target_dot=0;
  float avoid_final_count=0, final_count=0;
  float dot_gap;
  int avoid_ok=0;


  if(avoid_number > AVOID_NUMBER_MAX){
    avoid_ok=1;
  }

  for (int i = 0; i < avoid_number; i++) {
    avoid_target_dot=avoid_dot[i];
    int avoid_lidar_count=0;
    for (int j = 0; j < avoid_number; j++) {
      dot_gap=fabs(avoid_dot[j]-avoid_target_dot);
      if(dot_gap>180){
        dot_gap=360-dot_gap;
      }
      if(dot_gap<10){
        avoid_lidar_count++;
      }
    }

    if(avoid_lidar_count >= avoid_final_count){
      avoid_final_count = avoid_lidar_count;
      avoid_final_dot = avoid_target_dot;
    }
  }

  for (int i = 0; i < dot_number; i++) {
    target_dot=extract_dot[i];
    int lidar_count=0;
    for (int j = 0; j < dot_number; j++) {
      dot_gap=fabs(extract_dot[j]-target_dot);

      if(avoid_ok){
        avoid_gap1 = fabs(extract_dot[i]-avoid_final_dot);
        if(avoid_gap1>180) avoid_gap1 = 360 - avoid_gap1;

        avoid_gap2 = fabs(extract_dot[j]-avoid_final_dot);
        if(avoid_gap2>180) avoid_gap2 = 360 - avoid_gap2;

        if(avoid_gap1 >AVOID_GAP && avoid_gap2 >AVOID_GAP){
          if(dot_gap>180){
            dot_gap=360-dot_gap;
          }
          if(dot_gap<10){
            lidar_count++;
          }
        }
      }
      else{
        if(dot_gap>180){
          dot_gap=360-dot_gap;
        }
        if(dot_gap<10){
          lidar_count++;
        }
      }
    }

    if(lidar_count >= final_count){
      final_count = lidar_count;
      final_dot = target_dot;
    }
  }

  final_angle = (float)final_dot * M_PI / 180.0; // 0 ~ 2PI
  if(final_angle>M_PI) final_angle=final_angle-2*M_PI; // -PI ~ PI

  avoid_angle = (float)avoid_final_dot * M_PI / 180.0; // 0 ~ 2PI
  if(avoid_angle>M_PI) avoid_angle=avoid_angle-2*M_PI; // -PI ~ PI

  double theta_1 = final_angle * 180 / M_PI;
  double theta_2 = object_angle * 180 / M_PI;
  double test_1, test_2, theta_3, theta_gap;
  float angle_error;

  if(theta_1 < 0){
    test_1 = theta_1 + 360;
  }
  else{
    test_1 = theta_1;
  }
  if(theta_2 < 0){
    test_2 = theta_2 + 360;
  }
  else{
    test_2 = theta_2;
  }
  //0 ~ 360
  theta_gap = abs(test_1-test_2);

  if(theta_gap > 180){
    if(test_1 <= test_2){
      if(avoid_ok==1){
        theta_3 = test_2 + (360-theta_gap);
      }
      else{
        if(distance_error<DISTANCE_ERROR_MAX){
          theta_3 = test_2 + (360-theta_gap)*3/4;
        }
      }
    }
    else if(test_2 < test_1){
      if(avoid_ok==1){
        theta_3 = test_1;
      }
      else{
        if(distance_error<DISTANCE_ERROR_MAX){
          theta_3 = test_1 + (360-theta_gap)/4;
        }
      }
    }
  }
  else{
    if(test_1 <= test_2){
      if(avoid_ok==1){
        theta_3 = test_1;
      }
      else{
        if(distance_error<DISTANCE_ERROR_MAX){
          theta_3 = test_1+theta_gap/4;
        }
      }
    }
    else if(test_2 < test_1){
      if(avoid_ok==1){
        theta_3 = test_2+theta_gap;
      }
      else{
        if(distance_error<DISTANCE_ERROR_MAX){
          theta_3 = test_2+theta_gap*3/4;
        }
      }
    }
  }

  if(distance_error>(sqrt(pow(object_x,2)+pow(object_y,2))-0.3)){
    theta_3=0;
  }

  if(theta_3 > 180) theta_3 = theta_3 - 360;

  middle_angle = theta_3 * M_PI /180;

  if(avoid_ok){
    x=(int)(350*cos(avoid_angle));
    y=-(int)(350*sin(avoid_angle));
    line(plotImage, Point(400,400), Point(400+x,400+y), Scalar(0,0,255), 3);
  }

  if(final_count!=0){
    x=(int)(350*cos(final_angle));
    y=-(int)(350*sin(final_angle));
    line(plotImage, Point(400,400), Point(400+x,400+y), Scalar(0,255,0), 3);
    x=(int)(350*cos(middle_angle));
    y=-(int)(350*sin(middle_angle));
    line(plotImage, Point(400,400), Point(400+x,400+y), Scalar(0,255,255), 6);
  }

  x=(int)(350*cos(object_angle));
  y=-(int)(350*sin(object_angle));
  line(plotImage, Point(400,400), Point(400+x,400+y), Scalar(255,255,0), 3);

  x=(int)(350*cos(plot_anglehat));
  y=-(int)(350*sin(plot_anglehat));
  line(plotImage, Point(400,400), Point(400+x,400+y), Scalar(255,0,0), 5);

  for (int i = 0; i < 360; i++) {
    //cout << "i:" << i << "    ranges " << msg -> ranges[i] << " intensities " << msg -> intensities[i]<< endl;
  }
  if(robotMode==LIDAR_MODE){
    //imshow("plotImage",plotImage);
    sensor_msgs::ImagePtr lidar_msg;
    lidar_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", plotImage).toImageMsg();
    lidar_pub.publish(lidar_msg);
  }

  waitKey(WAITKEYSIZE);

  if(angle_hat < 0) angle_hat=angle_hat+2*M_PI;
  if(middle_angle < 0) middle_angle=middle_angle+2*M_PI;

  angle_error = middle_angle-angle_hat;

  if(angle_error>M_PI) angle_error=angle_error-2*M_PI;
  if(angle_error<-M_PI) angle_error=angle_error+2*M_PI;
  cout << angle_error << endl;
  /***********************publish*********************/
  if(robotMode!=LIDAR_MODE){
    sherlotics::LIDARtoCENTER mode_msg;
    mode_msg.data = robotMode;
    mode_pub.publish(mode_msg);
  }

  sherlotics::LINEtoPID send_msg;
  send_msg.data = 300*angle_error;
  if(robotMode==LIDAR_MODE) pub.publish(send_msg);

}

void rpmCallback(const std_msgs::UInt16::ConstPtr& msg )
{
  //cout << "msg2  " << msg->data << endl;
}

int main(int argc, char **argv)
// 노드 메인 함수
{
  ros::init(argc, argv, "lidarPart_node"); // 노드명 초기화

  std::ifstream file2("/home/m/initmode.txt");
  file2 >> robotMode;
  file2.close();

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  pub = nh.advertise<sherlotics::LINEtoPID>("LINEtoPID_msg", QUEUESIZE);
  mode_pub = nh.advertise<sherlotics::LIDARtoCENTER>("LIDARtoCENTER_msg", QUEUESIZE);
  lidar_pub = it.advertise("LIDARtoIMSHOW_msg", QUEUESIZE);

  ros::Subscriber mode_sub = nh.subscribe("CENTERtoLIDAR_msg", QUEUESIZE, modeCallback);
  ros::Subscriber scan_sub = nh.subscribe("scan", QUEUESIZE, scanCallback);
  ros::Subscriber rpms_sub = nh.subscribe("rpms", QUEUESIZE, rpmCallback);
  ros::Subscriber odom_sub = nh.subscribe("odom", QUEUESIZE, odomCallback);

  ros::spin();

  return 0;
}
