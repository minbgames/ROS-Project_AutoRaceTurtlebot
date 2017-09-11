#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sherlotics/variable.hpp>

using namespace cv;

Mat image_raw;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "videoPart_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub=it.advertise("original_msg", QUEUESIZE);

  VideoCapture cap("/home/m/Desktop/test_1280.mp4"); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
      return -1;

  while(1){
    cap >> image_raw;

    resize( image_raw, image_raw, Size( WIDTH_SIZE, HEIGHT_SIZE ), 0, 0, CV_INTER_CUBIC );

    sensor_msgs::ImagePtr originalMsg;
    originalMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_raw).toImageMsg();
    pub.publish(originalMsg);
    waitKey(WAITKEYSIZE);
  }

  return 0;
}
