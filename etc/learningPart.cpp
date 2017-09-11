#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sherlotics/variable.hpp>
#include <iostream>
#include <fstream>
#include <cstdlib>

#define RESIZED_SIZE 100
#define DATA_COUNT 1
#define WEIGHT_COUNT RESIZED_SIZE * RESIZED_SIZE
#define OBJECT_COUNT 3

using namespace cv;

Mat traffic_to_learning_image;

float x_train[DATA_COUNT][WEIGHT_COUNT];
float W[OBJECT_COUNT][WEIGHT_COUNT];
float cost=0;
int pixel_index=0;

float Hypothesis_Make(int _DataIndex, int _ObjectIndex)
{
    float hypothesis_ =0;
    for (int k = 0; k < WEIGHT_COUNT; k++) {
        hypothesis_ += x_train[_DataIndex][k] * W[_ObjectIndex][k];
    }
    return hypothesis_;
}

void result_test(){
    float hypothesis[OBJECT_COUNT];
    float softmax[OBJECT_COUNT];
    float softmax_sum;
    float maximum_value=0;
    int sign_index=-1;
    for (int i = 0; i < DATA_COUNT; ++i) {

        softmax_sum=0;
        for (int j = 0; j < OBJECT_COUNT; j++) {
            hypothesis[j]=Hypothesis_Make(i,j);
            softmax_sum+=exp(hypothesis[j]/RESIZED_SIZE);
        }

        for (int j = 0; j < OBJECT_COUNT; j++) {
            softmax[j]=exp(hypothesis[j]/RESIZED_SIZE)/softmax_sum;

            ROS_INFO("%f ",softmax[j]);
        }

        for (int j = 0; j < OBJECT_COUNT; j++) {
            if(maximum_value<softmax[j]){
                maximum_value=softmax[j];
                sign_index=j+1;
            }
        }
    }

    switch(sign_index){
      case 1:
          ROS_INFO("this is turnel sign");
          break;
      case 2:
          ROS_INFO("this is parking sign");
          break;
      case 3:
          ROS_INFO("this is not sign");
          break;
    }
}

void learningImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    traffic_to_learning_image = cv_bridge::toCvShare(msg, "mono8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }

  pixel_index=0;
  for (int i = 0; i < traffic_to_learning_image.rows; ++i) {
      for (int j = 0; j < traffic_to_learning_image.cols; ++j) {
          x_train[0][pixel_index]=(float)traffic_to_learning_image.at<uchar>(i,j)/255;
          pixel_index++;
      }
  }//x_train
  result_test();

  waitKey(WAITKEYSIZE);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "learningPart_node");

  srand((unsigned int)time(NULL));

  std::ifstream file("/home/m/catkin_ws/src/sherlotics/data/example.txt");

  for(int j=0; j<OBJECT_COUNT; j++) {
      for(int k=0; k<WEIGHT_COUNT; k++) {
          file >> W[j][k];
          //std::cout << W[j][k];
      }
  }

  file.close();


  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("TRAFFICtoLEARNING_msg", QUEUESIZE, learningImageCallback);

  ros::spin();

  return 0;
}
