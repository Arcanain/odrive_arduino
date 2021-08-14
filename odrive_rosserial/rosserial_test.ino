/* Odrive */
//#include <ODriveTool.h>
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
/******/

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
//HardwareSerial& odrive_serial = Serial1;

// Serial1はrosserialの方で使用されるため、ArduinoとOdriveはSerial2でUART通信を行う
// pin 17: RX - connect to ODrive TX
// pin 16: TX - connect to ODrive RX
HardwareSerial& odrive_serial = Serial2;

// ODrive object
//ODriveTool odrive(Serial1);
ODriveArduino odrive(odrive_serial);

void messageCb(const geometry_msgs::Twist& msg);

/* ROS */
ros::NodeHandle  nh;

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);

/***********************************************************************
 * Global variables
 **********************************************************************/
const int kv = 16;
const int encoder_cpr = 90;
const float pi = 3.14159262;

float w_r = 0.0;
float w_l = 0.0;

//wheel_rad is the wheel radius ,wheel_sep is
float wheel_rad = 0.085;
float wheel_sep = 0.32;

float speed_ang = 0.0;
float speed_lin = 0.0;

float vel1,vel2;

bool calb_flag = true;

void ros_init()
{
    //nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub);
}

void odrive_calibration()
{
  calb_flag = false;
  // axis0のみcalibration
  char c = '0';
  int motornum = 0;
  int requested_state;

  requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
  //Serial2 << "Axis" << c << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(motornum, requested_state, true)) return;

  requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  //Serial2 << "Axis" << c << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(motornum, requested_state, true, 25.0f)) return;

  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  //Serial2 << "Axis" << c << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(motornum, requested_state, false /*don't wait*/)) return;
}

void setup() {
  // ODrive uses 115200 baud
  Serial2.begin(115200);

  ros_init();
}

void loop() {
 
  if (calb_flag == true) {
    odrive_calibration();
  }

  nh.spinOnce();
  delay(500);
}

void messageCb(const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));

  //vel1 = w_r;
  //vel2 = w_l;
  vel1 = 2.0f;
  vel2 = 2.0f;
  odrive.SetVelocity(0, vel1);
  odrive.SetVelocity(1, vel2);
}