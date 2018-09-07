#include "../include/underpan/underpan.h"

/**
 * @brief 底盘控制节点主程序
 */

// 创建Jarvis类的dalu对象，并调用初始化函数.
static underpan::Jarvis dalu;

void cmdCallback(const geometry_msgs::Twist& cmd_input);

/**
 * @brief  底盘控制节点，订阅控制指令，按照协议格式产生控制命令，并通过串口发送至底盘
 * @par
 */

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "underpan_ctrl");
  ros::NodeHandle nh;

  if(!dalu.init())
    ROS_ERROR("dalu underpan initialized failed.");
  ROS_INFO("dalu underpan initialized successful.");

  // 发布odom话题，消息类型为nav_msgs/odometry，话题名称，发布频率
  ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 20);

  // 订阅/cmd_vel话题,的控制指令
  ros::Subscriber sub = nh.subscribe("teleop/cmd_vel", 50, cmdCallback);

  // 50Hz循环运行
  ros::Rate loop_rate(50);

while (ros::ok())
{
  // 接收串口数据
  dalu.spinOnce();

  // 周期执行回调函数
  ros::spinOnce();
  // 休眠
  loop_rate.sleep();
}

	return 0;
}

/**
 * @brief 控制消息的回调函数,接收到控制消息进行处理.
 * @par   键盘发布的Twist类型的速度指令.
 */

void cmdCallback(const geometry_msgs::Twist& cmd_input)
{
  // 声明控制指令数据存储数组
  unsigned char ctrlCmdData[19] = {'0'};
  // 定义速度值临时变量
  float linear_speed_x_temp  = 0;
  float linear_speed_y_temp  = 0;
  float angular_speed_temp   = 0;

  // 获取从/cmd_vel话题发布的速度信息
  linear_speed_x_temp = cmd_input.linear.x;
  linear_speed_y_temp = cmd_input.linear.y;
  angular_speed_temp  = cmd_input.angular.z;

  // 生成底盘控制指令，返回数组指针，将地址存储在临时变量中，
  unsigned char *ctrlCmdData_temp = dalu.getCtrlCmd(
        underpan::WHEELMODE_NORMAL, linear_speed_x_temp, linear_speed_y_temp, angular_speed_temp);
  for(int i = 0;i < 19; i++)
  {
    ctrlCmdData[i] = *ctrlCmdData_temp;
    ctrlCmdData_temp++;
  }

  // 通过串口发送数据，boost::asio::buffer()中的数据类型为数组名
  boost::asio::write(underpan::serialPort, boost::asio::buffer(ctrlCmdData));
}
