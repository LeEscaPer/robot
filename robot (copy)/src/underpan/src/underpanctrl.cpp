#include <vector>
#include "../include/underpan/underpan.h"

using namespace std;
using namespace boost::asio;

namespace underpan{

//Jarvis类的构造函数
Jarvis::Jarvis()
{
}

//Jarvis类的析构函数
Jarvis::~Jarvis()
{
}

/**
 * @brief  Jarvis::init  Jarvis Robot类的初始化函数，完成串口配置等
 * @return 真值初始化完成
 */

bool Jarvis::init()
{
  underpan::serialPort.set_option(boost::asio::serial_port::baud_rate(115200));                                           //波特率
  underpan::serialPort.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));  //流控
  underpan::serialPort.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));              //校验位
  underpan::serialPort.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));         //停止位
  underpan::serialPort.set_option(boost::asio::serial_port::character_size(8));                                           //数据位

  ros::Time::init();
	current_time_ = ros::Time::now();
  last_time_    = ros::Time::now();

  return true;
}

/**
 * @brief   Jarvis::getFeedbackInfo 提取串口数据并提取底盘反馈信息
 * @return
 */

struct underpanInfo Jarvis::getFeedbackInfo()
{
  unsigned char i;
  unsigned char recchecksum;      // 声明BCC校验和暂存变量
  unsigned char recbuf[28];       // 声明串口接收缓存数组
  unsigned char recheader[2];     // 定义接收数据帧头
  underpan::underpanInfo dalu;    // 创建大陆底盘信息结构体变量
  underpan::Check checkBCC;       // 声明Check类的对象

  // 读取串口数据
  boost::asio::read(underpan::serialPort, boost::asio::buffer(recbuf));
  //ros::Time curr_time;

  // 提取并校验信息帧头,帧头有误数据丢弃
  for (i = 0; i < 2; i++)
  {
    recheader[i] = recbuf[i];
  }
  if (recheader[0] != underpan::FRAME_HEADER_H || recheader[1] != underpan::FRAME_HEADER_L)
	{
    ROS_ERROR("Received message header error!");    //帧头错误报告
  }

  // 提取除校验位外的数据并进行BCC校验
  recchecksum = checkBCC.getSUM(recbuf,(sizeof(recbuf)-1));
  if (recchecksum != recbuf[27])
  {
    ROS_ERROR("Received data check sum error!");   //数据校验错误报告
  }

  // 数据校验无误，则提取底盘反馈信息，异常信息
  for(i = 0; i < sizeof(recbuf); i++)
  {
    // 提取底盘车轮码盘数
    dalu.rightFront.encoderData  = recbuf[i + 3];
    dalu.rightMiddle.encoderData = recbuf[i + 5];
    dalu.rightRear.encoderData   = recbuf[i + 7];
    dalu.leftFront.encoderData   = recbuf[i + 9];
    dalu.leftMiddle.encoderData  = recbuf[i + 11];
    dalu.leftRear.encoderData    = recbuf[i + 13];

    // 提取底盘车轮电流值
    dalu.rightFront.currentData  = recbuf[i + 17];
    dalu.rightMiddle.currentData = recbuf[i + 18];
    dalu.rightRear.currentData   = recbuf[i + 19];
    dalu.leftFront.currentData   = recbuf[i + 20];
    dalu.leftMiddle.currentData  = recbuf[i + 21];
    dalu.leftRear.currentData    = recbuf[i + 22];

    // 提取状态信息
    dalu.stateMotor   = recbuf[i + 23];
    dalu.stateDriver  = recbuf[i + 24];
    dalu.stateEncoder = recbuf[i + 25];
    dalu.stateComm    = recbuf[i + 26];
  }

  return dalu;
}


/**
 * @brief  Jarvis类的控制命令产生函数，底盘控制信息.根据车轮模式，判断刹车模式或是正常，根据模式不同向缓存数组赋值.
 * @par    车轮模式、X轴线速度、Y轴线速度、Z轴角速度.
 * @return 按照协议格式产生控制命令.
 */

unsigned char *Jarvis::getCtrlCmd(unsigned char wheelmode,
    float linear_speed_x,
    float linear_speed_y,
    float angular_speed)
{
  static unsigned char sendbuffer[19] = {0};  // 定义数据存储数组并初始化
  underpan::Check checkBCC;                   // 声明校验类checkBCC对象

  for(int i = 0; i < 2; i++)                  // 写入帧固定数据，包括帧头位、标识位、帧长度
  {
    sendbuffer[i]   = FRAME_HEADER[i];
    sendbuffer[i+3] = FRAME_LABLE[i];
  }
  sendbuffer[2] = FRAME_LENGTH_CTRL;

  switch (wheelmode) {                        // 判断车轮模式，并写入对应的车轮模式值和速度值
  case WHEELMODE_BRAKE_TURNING:{              // 刹车模式，速度值为0
    sendbuffer[5] = WHEELMODE_BRAKE_TURNING;
    memset(sendbuffer + 5 , '\0', 12);
    break;
  }
  case WHEELMODE_NORMAL:{                     // 正常模式，设置速度值
    sendbuffer[5] = WHEELMODE_NORMAL;
    float *linear_speed_x_temp = (float*)(sendbuffer + 6);
    float *linear_speed_y_temp = (float*)(sendbuffer + 10);
    float *angular_speed_temp  = (float*)(sendbuffer + 14);
    *linear_speed_x_temp = linear_speed_x;
    *linear_speed_y_temp = linear_speed_y;
    *angular_speed_temp  = angular_speed;     // 右手坐标系 适配底盘角速度取反
    break;
  }
  default:{                                   // 默认短接刹车模式
    sendbuffer[5] = WHEELMODE_BRAKE_SHORTCIRCUIT;
    memset(sendbuffer + 5 , '\0', 12);
    break;
  }
  }

  // 计算并设置校验值
  unsigned char sum ;//= (unsigned char*)(sendbuffer + 18);
  sum = checkBCC.getSUM(sendbuffer, sizeof(sendbuffer));
  sendbuffer[18] = sum;

  return sendbuffer;

//#ifdef _DEBUG
  for (int i = 0; i < FRAME_LENGTH_CTRL; i++)
  {
    std::cout << std::hex << (int)sendbuffer[i] << " ";
  }
  std::cout << std::endl;
//#endif
}

/**
 * @brief
 * @par
 */

bool Jarvis::spinOnce()
{
  struct underpanInfo dalu;
  // 读取底盘信息
//  dalu = getFeedbackInfo();

/*
  // 发布里程计消息
  nav_msgs::Odometry msgl;
  msgl.header.stamp = current_time_;
  msgl.header.frame_id = "odom";

  msgl.pose.pose.position.x = x_;
  msgl.pose.pose.position.y = y_;
  msgl.pose.pose.position.z = 0.0;
  msgl.pose.pose.orientation = odom_quat;
  msgl.pose.covariance = odom_pose_covariance;

  msgl.child_frame_id = "base_footprint";
  msgl.twist.twist.linear.x = vx_;
  msgl.twist.twist.linear.y = vy_;
  msgl.twist.twist.angular.z = vth_;
  msgl.twist.covariance = odom_twist_covariance;
  */
}

/**
 * @brief  里程计算法，计算车辆里程算法
 * @par    编码器
 */

unsigned int Jarvis::calOdometry(unsigned short int encoderInfo)
{
    unsigned int odometer;
    odometer = (2*PI*UNDERPAN_RADIUS/60)*encoderInfo;
    return odometer;
}

/**
 * @brief Check::Check构造函数
 */

Check::Check()
{
}

/**
 * @brief Check::~Check析构函数
 */

Check::~Check()
{
}


/**
 * @brief Check类的BCC校验求累积和函数
 * @par   待校验数组，长度
 */

unsigned char Check::getSUM(unsigned char *buf, int datalength)
{
  unsigned char sum = 0x00;
  for (int i = 0; i < datalength - 1; i++)
  {
    sum ^= buf[i];
  }
  return sum;
}

}
