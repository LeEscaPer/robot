#ifndef UNDERPAN_H
#define UNDERPAN_H

/**
 * @brief 底盘控制的头文件
 */

#include <boost/asio.hpp>                      //boost库函数

#include <ros/ros.h>                           //ROS头文件
#include <ros/time.h>

#include "std_msgs/String.h"                   //ROS定义的String数据类型
#include <geometry_msgs/Twist.h>               //ROS定义的Twist消息数据类型
#include <nav_msgs/Odometry.h>                 //ROS定义的Odometry里程计数据类型
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

namespace underpan{

// 创建串口类的对象
static boost::asio::io_service iosev;
static boost::asio::serial_port serialPort(iosev, "/dev/ttyUSB0");

/**
 * @brief 定义底盘串口传输协议帧、车轮模式和底盘反馈信息
 */
const unsigned char FRAME_HEADER_H     = 0xAA;                // 帧头高位
const unsigned char FRAME_HEADER_L     = 0x55;                // 帧头低位
const unsigned char FRAME_HEADER[2]    = {0xAA, 0x55};        // 帧头数组
const unsigned char FRAME_LABLE_H      = 0x12;                // 大标示位
const unsigned char FRAME_LABLE_L      = 0x01;                // 小标示位
const unsigned char FRAME_LABLE[2]     = {0x12, 0x01};        // 标示位数组
const unsigned char FRAME_LENGTH_CTRL  = 0x0F;                // 控制指令帧长度
const unsigned char FRAME_LENGTH_INFO  = 0x18;                // 反馈信息帧长度

const unsigned char WHEELMODE_NORMAL             = 0x00;      // 正常模式
const unsigned char WHEELMODE_BRAKE_LEFT_FRONT   = 0x08;      // 左前刹车
const unsigned char WHEELMODE_BRAKE_LEFT_MID     = 0x10;      // 左中刹车
const unsigned char WHEELMODE_BRAKE_LEFT_RARE    = 0x20;      // 左后刹车
const unsigned char WHEELMODE_BRAKE_RIGHT_FRONT  = 0x01;      // 右前刹车
const unsigned char WHEELMODE_BRAKE_RIGHT_MID    = 0x02;      // 右中刹车
const unsigned char WHEELMODE_BRAKE_RIGHT_RARE   = 0x04;      // 右后刹车
const unsigned char WHEELMODE_BRAKE_TURNING      = 0x3F;      // 转向刹车
const unsigned char WHEELMODE_BRAKE_SHORTCIRCUIT = 0xFF;      // 短接刹车

const unsigned char STATE_MOTOR_NORMAL              = 0X00;      // 电机无异常
const unsigned char STATE_MOTOR_ERR_RIGHT_FRONT     = 0x01;      // 右前轮电机异常
const unsigned char STATE_MOTOR_ERR_RIGHT_MID       = 0x02;      // 右中轮电机异常
const unsigned char STATE_MOTOR_ERR_RIGHT_RARE      = 0x04;      // 右后轮电机异常
const unsigned char STATE_MOTOR_ERR_LEFT_FRONT      = 0x08;      // 左前轮电机异常
const unsigned char STATE_MOTOR_ERR_LEFT_MID        = 0x10;      // 左中轮电机异常
const unsigned char STATE_MOTOR_ERR_LEFT_RARE       = 0x30;      // 左后轮电机异常

const unsigned char STATE_DRIVER_NORMAL             = 0x00;      // 驱动器无异常
const unsigned char STATE_DRIVER_ERR_RIGHT_FRONT    = 0x01;      // 右前轮驱动器异常
const unsigned char STATE_DRIVER_ERR_RIGHT_MID      = 0x02;      // 右中轮驱动器异常
const unsigned char STATE_DRIVER_ERR_RIGHT_RARE     = 0x04;      // 右后轮驱动器异常
const unsigned char STATE_DRIVER_ERR_LEFT_FRONT     = 0x08;      // 左前轮驱动器异常
const unsigned char STATE_DRIVER_ERR_LEFT_MID       = 0x10;      // 左中轮驱动器异常
const unsigned char STATE_DRIVER_ERR_LEFT_RARE      = 0x30;      // 左后轮驱动器异常

const unsigned char STATE_ENCODER_NORMAL            = 0x00;      // 码盘无异常
const unsigned char STATE_ENCODER_ERR_RIGHT_FRONT   = 0x01;      // 右前轮码盘异常
const unsigned char STATE_ENCODER_ERR_RIGHT_MID     = 0x02;      // 右中轮码盘异常
const unsigned char STATE_ENCODER_ERR_RIGHT_RARE    = 0x04;      // 右后轮码盘异常
const unsigned char STATE_ENCODER_ERR_LEFT_FRONT    = 0x08;      // 左前轮码盘异常
const unsigned char STATE_ENCODER_ERR_LEFT_MID      = 0x10;      // 左中轮码盘异常
const unsigned char STATE_ENCODER_ERR_LEFT_RARE     = 0x30;      // 左后轮码盘异常

const unsigned char STATE_COMM_NORMAL               = 0x00;      // 通讯正常
const unsigned char STATE_COMM_ERR                  = 0x01;      // 通讯异常

/**
 * @brief 定义底盘物理参数
 */
const double UNDERPAN_RADIUS                  = 150.00;     //定义底盘车轮直径
const unsigned short int UNDERPAN_ENCODER_SUM = 400;        //定义码盘总数
const float PI = 3.14;                                      //定义圆周率
//const double ROBOT_LENGTH = 210.50;

/**
 * @brief 声明底盘信息数据结构
 */

// 定义车轮信息,每个轮子包括码盘数、电流值、状态信息.
struct wheelInfo
{
  unsigned short int encoderData;  // 码盘数：0～400
  unsigned char currentData;       // 电流值：0～255
};

// 定义六轮驱动底盘反馈信息
struct underpanInfo
{
  struct wheelInfo leftFront;    // 左前
  struct wheelInfo leftMiddle;   // 左中
  struct wheelInfo leftRear;     // 左后
  struct wheelInfo rightFront;   // 右前
  struct wheelInfo rightMiddle;  // 右中
  struct wheelInfo rightRear;    // 右后
  unsigned char stateMotor;      // 电机状态
  unsigned char stateDriver;     // 驱动器状态
  unsigned char stateEncoder;    // 码盘状态
  unsigned char stateComm;       // 通讯状态
};

/**
 * @brief The Jarvis class
 */

class Jarvis
{
public:
  Jarvis();
  ~Jarvis();
  bool init();
  bool spinOnce();
  struct underpanInfo getFeedbackInfo();
  unsigned char *getCtrlCmd(unsigned char wheelmode, float linear_speed_x, float linear_speed_y, float angular_speed);
  unsigned int calOdometry(unsigned short int encoderInfo);

private:
  ros::Time current_time_, last_time_;
//  struct underpanInfo underpanInfo;
//  tf::TransformBroadcaster odom_broadcaster_;
};

/**
 * @brief The Check class
 */

class Check
{
public:
  Check();
  ~Check();
  unsigned char getSUM(unsigned char *buf, int datalength);

private:
  /* data */
};

}

#endif /* UNDERPAN_H */
