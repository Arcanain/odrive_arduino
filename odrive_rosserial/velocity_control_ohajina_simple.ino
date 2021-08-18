/* Board  : Arduino Mega 2560
 * Author : Ramune6110
 * Data   : 2021 08/18(未検証 : 2021 08/19に検証予定)
 * ********************************************************************************
 * Topic
 * Publish  | cmd_vel
 * ********************************************************************************
 */
#include "ODriveTool.h"

/* ROS */
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>

// ODrive object
// Serial1はrosserialの方で使用されるため、ArduinoとOdriveはSerial2でUART通信を行う
// pin 17: RX - connect to ODrive TX
// pin 16: TX - connect to ODrive RX
ODriveTool odrive(Serial2);

void messageCb(const geometry_msgs::Twist& msg);

/* ROS */
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);

/***********************************************************************
 * Global variables
 **********************************************************************/
const int kv = 16;
const int encoder_cpr = 90;
const float pi = 3.14159262f;

float w_r = 0.0f;
float w_l = 0.0f;

//wheel_rad is the wheel radius ,wheel_sep is
float wheel_rad = 0.085f;
float wheel_sep = 0.32f;

float speed_ang = 0.0f;
float speed_lin = 0.0f;

float vel1 = 0.0f;
float vel2 = 0.0f;

void ros_init()
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub);
}

void setup() {
  odrive.odrive_reboot();
  for(int i = 0; i < 2 ; i++){
    //odrive.odrive_init(i);
    odrive.odrive_init(i, 5000.0f, 20.0f);
    delay(300);
  }
  ros_init();
}

void loop() {
  nh.spinOnce();
  delay(500);
}

void messageCb(const geometry_msgs::Twist& msg){
  speed_lin = msg.linear.x;
  speed_ang = msg.angular.z;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));

  //vel1 = w_r;
  //vel2 = w_l;
  vel1 = 2.0f;
  vel2 = 2.0f;
  odrive.SetVelocity(0, vel1);
  odrive.SetVelocity(1, vel2);
}