/* Board  : Arduino Mega 2560
 * Author : Ramune6110
 * Data   : 2021 08/18(未検証 : 2021 08/19に検証予定)
 * ********************************************************************************
 * Topic
 * Publish  | cmd_vel
 * Publish  | position_pub
 * Publish  | velocity_pub
 * Publish  | voltage_pub
 * Sbscribe | cmd_vel
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

std_msgs::Int32MultiArray position_data;
ros::Publisher position_pub("/position", &position_data);

std_msgs::Float32MultiArray velocity_data;
ros::Publisher velocity_pub("/velocity", &velocity_data);

std_msgs::Float32 voltage_data;
ros::Publisher voltage_pub("/voltage", &voltage_data);

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

void array_init(std_msgs::Int32MultiArray& data, int array){
    data.data_length = array;
    data.data = (int32_t *)malloc(sizeof(int32_t)*array);
    for(int i = 0; i < array ;i++){
        data.data[i] = 0;
    }
}

void array_init(std_msgs::Float32MultiArray& data, int array){
    data.data_length = array;
    data.data = (float *)malloc(sizeof(float)*array);
    for(int i = 0; i < array ;i++){
        data.data[i] = 0;
    }
}

void ros_init()
{
    array_init(position_data,2);
    array_init(velocity_data,2);
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(position_pub);
    nh.advertise(velocity_pub);
    nh.advertise(voltage_pub);
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
  voltage_data.data = odrive.get_voltage();
  voltage_pub.publish(&voltage_data);

  for(int motor = 0; motor < 2 ;motor++){
    position_data.data[motor] = odrive.get_position(motor);
    velocity_data.data[motor] = odrive.get_velocity(motor);
  }
  position_pub.publish(&position_data);
  velocity_pub.publish(&velocity_data);
  
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