#include <ros/ros.h>
#include <sherlotics/LINEtoPID.h>
#include <sherlotics/BLOCKLIGHTtoPID.h>
#include <sherlotics/CENTERtoPID.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <sherlotics/variable.hpp>
#include <iostream>
#include <ctime>

using namespace std;

ros::Publisher pub;

float line_gap=0;
float scale;
float gap_error=0;
float p_error=0;
float error_integral=0;
float filtering_data=0;
float error_total;
float steering;
float filtering_error=0;
//initial value setting

int obstacle_ok=0;
float linear_velocity=0;
float angular_velocity=0;

int light_flag_green = 1, light_flag_red = 1;
double time_gap_green = 0, time_gap_red = 0;
time_t startTime_green=0, startTime_red=0, currentTime=0;

void msgCallback1(const sherlotics::BLOCKLIGHTtoPID::ConstPtr& msg ) //1 and 0
{
  obstacle_ok=msg->data;   // 1_block bar, 2_light red, 3_light green
}

void msgCallback3(const sherlotics::CENTERtoPID::ConstPtr& msg ) //mode
{
  robotMode=msg->data;
}

void msgCallback2(const sherlotics::LINEtoPID::ConstPtr& msg )
{
  currentTime = clock();

  if(light_flag_green == 1){
    if(obstacle_ok == 3){
      light_flag_green=0;
      startTime_green = clock();
    }
  }

  if(light_flag_green==0){
    time_gap_green= (float)(currentTime-startTime_green)/10000;
    cout << "time_gap_green: " << time_gap_green << endl;
    if(time_gap_green<20){
      obstacle_ok = 4; // green
    }
    else{
      light_flag_green=-1;
    }
  }

  if(light_flag_red == 1){
    if(obstacle_ok == 2){
      if(light_flag_green==-1){
        light_flag_red=0;
        startTime_red = clock();
      }
    }
  }

  if(light_flag_red==0){
    time_gap_red= (float)(currentTime-startTime_red)/10000;
    cout << "time_gap_red: " << time_gap_red << endl;
    if(time_gap_red<20){
      obstacle_ok = 5; // red
    }
    else{
      light_flag_red=-1;
    }
  }

  linear_velocity=INITIALVELOCITY;

  line_gap=(float)(msg->data);

  if(line_gap > -200 && line_gap <200){
    Kp=1.3;
  }
  else{
    Kp=1.7;
  }

  if(robotMode==NORMAL_TO_PARKING){
    linear_velocity=0.03; //0.08
    Kp=0.2; //0.5 0.7
  }

  if(robotMode==LIDAR_MODE){
    linear_velocity=0.1;
    Kp=4;
  }

  if(disp_obstacle){
    cout << "obstacle_ok: " << obstacle_ok <<endl;
  }

  if(obstacle_ok==4){
    linear_velocity=0.02;
  }

  if((obstacle_ok==1) || (obstacle_ok==5)){
    linear_velocity=0;
    angular_velocity=0;
  }
  else{
    scale = line_gap*0.01;
    filtering_data = (1-alpha)*filtering_data+alpha*scale; //filtering

    if(disp_lineGap){
      cout << "lineGap: " << filtering_data <<endl;
    }
    gap_error=filtering_data-goal; //present value - 0

    error_total=Kp*gap_error+Ki*error_integral+Kd*(p_error-gap_error)/cycle_time;
    error_integral=error_integral+gap_error*cycle_time;

    if(error_integral > MAX_INTEGRAL) error_integral = MAX_INTEGRAL;
    if(error_integral < MIN_INTEGRAL) error_integral = MIN_INTEGRAL;

    p_error = gap_error;

    // if(error_total > MAX_ERROR) error_total = MAX_ERROR;
    // if(error_total < MIN_ERROR) error_total = MIN_ERROR;

    if(disp_error_integral) cout << "error_integral:  " << error_integral << endl;
    if(disp_error_total) cout << "error_total:  " << error_total <<endl;
    filtering_error = (1-error_alpha)*filtering_error+error_alpha*error_total;
    angular_velocity = filtering_error*SCALE_VALUE;
    /***********linear_velocity,angular_velocity*********************/
  }

  if(disp_angular_velocity) cout << "angular_velocity" << angular_velocity << endl;
  if(disp_linear_velocity) cout << "linear_velocity" << linear_velocity << endl;

  std_msgs::Float32MultiArray PID_msg;
  PID_msg.data.clear();

  float sendData[2] = {linear_velocity,angular_velocity};

  for (int i = 0; i < 2; i++)
	{
		PID_msg.data.push_back(sendData[i]);
	}
  pub.publish(PID_msg);
}

int main(int argc, char **argv)
// 노드 메인 함수
{
  ros::init(argc, argv, "PIDpart_node"); // 노드명 초기화
  ros::NodeHandle nh;

  ros::Subscriber sub1 = nh.subscribe("BLOCKLIGHTtoPID_msg", QUEUESIZE, msgCallback1);
  ros::Subscriber sub2 = nh.subscribe("LINEtoPID_msg", QUEUESIZE, msgCallback2);
  ros::Subscriber sub3 = nh.subscribe("CENTERtoPID_msg", QUEUESIZE, msgCallback3);
  pub = nh.advertise<std_msgs::Float32MultiArray>("TELEOP_msg", QUEUESIZE);

  ros::spin();

  return 0;
}
