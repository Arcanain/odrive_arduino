/* Board  : Arduino Mega 2560
 * Author : Ramune6110
 * Data   : 2021 08/26(ROS通信成功！2輪とも回転した！)
 * 各種daley()を無くす事で初期通信時の赤いエラーが消え、通信がより滑らかになった！
 * ros_odrive内に生成したkey_teleop.pyを使用して八方向モーター制御が出来た！(rosrun ros_odrive key_teleop.py)
 * ********************************************************************************
 * Topic
 * Publish  | cmd_vel
 * Sbscribe | cmd_vel
 * ********************************************************************************
 */
/* Odrive */
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

/* ROS */
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <ArduinoHardware.h>
#include <geometry_msgs/Twist.h>

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
//HardwareSerial& odrive_serial = Serial1;

// Arduino Mega or Due - Serial2
// Serial1はrosserialの方で使用されるため、ArduinoとOdriveはSerial2でUART通信を行う
// pin 17: RX - connect to ODrive TX GPIO 1
// pin 16: TX - connect to ODrive RX GPIO 2
HardwareSerial& odrive_serial = Serial2;

// ODrive object
ODriveArduino odrive(odrive_serial);

void messageCb(const geometry_msgs::Twist& msg);

/* ROS */
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);

std_msgs::Float32MultiArray velocity_data;
ros::Publisher velocity_pub("/velocity", &velocity_data);

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
  nh.advertise(velocity_pub);
}

void odrive_calibration()
{
  int motornum0 = 0;
  int motornum1 = 1;
  int requested_state0;
  int requested_state1;

  requested_state0 = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(!odrive.run_state(motornum0, requested_state0, false /*don't wait*/)) return;
  
  requested_state1 = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(!odrive.run_state(motornum1, requested_state1, false /*don't wait*/)) return;
}

void setup() {
  // ODrive uses 115200 baud(Odriveは115200でないと動かない様子)
  odrive_serial.begin(115200);
 //delay(3000);
  
  // moterキャリブレーション
  odrive_calibration();
 //delay(300);

  ros_init();
}

void loop() {
  nh.spinOnce();
 //delay(10);
}

void messageCb(const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));

  vel1 = w_r * 0.3;
  vel2 = w_l * 0.3;

  //velに送る値の単位はrpm(rad/s → rpmに変換する必要あり)
  odrive.SetVelocity(0, vel1);
  odrive.SetVelocity(1, vel2);
}