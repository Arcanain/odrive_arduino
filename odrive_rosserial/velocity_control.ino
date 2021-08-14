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
HardwareSerial& odrive_serial = Serial1;

// ODrive object
//ODriveTool odrive(Serial1);
ODriveArduino odrive(odrive_serial);

void messageCb(const geometry_msgs::Twist& msg);

//void odrive_cb(const std_msgs::Float32MultiArray& joy_msg);

/* ROS */
ros::NodeHandle  nh;

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);

/*
ros::Subscriber<std_msgs::Float32MultiArray> joy_sub("/joy_array", odrive_cb);

std_msgs::Int32MultiArray position_data;
ros::Publisher position_pub("/position", &position_data);

std_msgs::Float32MultiArray velocity_data;
ros::Publisher velocity_pub("/velocity", &velocity_data);

std_msgs::Float32 voltage_data;
ros::Publisher voltage_pub("/voltage", &voltage_data);
*/

/******/

/***********************************************************************
 * Global variables
 **********************************************************************/
float w_r = 0.0;
float w_l = 0.0;

//wheel_rad is the wheel radius ,wheel_sep is
float wheel_rad = 0.085;
float wheel_sep = 0.32;

float speed_ang = 0.0;
float speed_lin = 0.0;

float vel1,vel2;

/*
void array_init(std_msgs::Int32MultiArray& data, int array){
    data.data_length=array;
    data.data=(int32_t *)malloc(sizeof(int32_t)*array);
    for(int i=0; i<array ;i++){
        data.data[i]=0;
    }
}

void array_init(std_msgs::Float32MultiArray& data, int array){
    data.data_length=array;
    data.data=(float *)malloc(sizeof(float)*array);
    for(int i=0; i<array ;i++){
        data.data[i]=0;
    }
}
*/

void ros_init()
{
    /*
    array_init(position_data,2);
    array_init(velocity_data,2);
    */
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub);

    /*
    nh.subscribe(joy_sub);
    nh.advertise(position_pub);
    nh.advertise(velocity_pub);
    nh.advertise(voltage_pub);
    */
}

void setup() {
  /*
  odrive.odrive_reboot();
  for(int i=0; i< 2 ; i++){
    odrive.odrive_init(i);
    delay(300);
  }
  */
  ros_init();
}

void loop() {
  /*
  voltage_data.data=odrive.get_voltage();
  voltage_pub.publish(&voltage_data);
  
  for(int motor=0; motor<2 ;motor++){
    position_data.data[motor]=motor->pos_setpoint
    velocity_data.data[motor]=motor->vel_setpoint
  }
  position_pub.publish(&position_data);
  velocity_pub.publish(&velocity_data);
  */

  nh.spinOnce();
  //delay(500);
}

void messageCb(const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));

  vel1 = w_r;
  vel2 = w_l;
  odrive.SetVelocity(0, vel1);
  odrive.SetVelocity(1, vel2);
}

/*
void odrive_cb(const std_msgs::Float32MultiArray& joy_msg){
    joy_msg.data[1] > 0 ? vel1=pow((joy_msg.data[1]*3.0),2)*(40000.0/9.0) : vel1=pow((joy_msg.data[1]*3.0),2)*(-40000.0/9.0);
    joy_msg.data[4] > 0 ? vel2=pow((joy_msg.data[4]*3.0),2)*(40000.0/9.0) : vel2=pow((joy_msg.data[4]*3.0),2)*(-40000.0/9.0);

    odrive.SetVelocity(0,vel1);
    odrive.SetVelocity(1,vel2);
}
*/