#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "../include/underpan/teleop.h"

class TeleopRobot
{
public:
  TeleopRobot();
  void keyLoop();

private:


  ros::NodeHandle nh_;
  float linear_speed_x, linear_speed_y, angular_speed, l_scale_, a_scale_;
  ros::Publisher twist_pub_;

};

TeleopRobot::TeleopRobot():
  linear_speed_x(0),
  linear_speed_y(0),
  angular_speed(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("teleop/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_node");
  TeleopRobot teleoprobot;

  signal(SIGINT,quit);

  teleoprobot.keyLoop();

  return(0);
}


void TeleopRobot::keyLoop()
{
  char c;
  bool dirty=false;
  float linear_speed_x_value = 0.3;
  float linear_speed_y_value = 0.3;
  float angular_speed_value  = 0.3;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");


  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_speed_x = linear_speed_y = angular_speed = 0;
    ROS_DEBUG("value: 0x%02X\n", c);

    switch(c){
    // 左转
    case teleop::KEYCODE_LEFT_ANGULAR:
      ROS_DEBUG("LEFT");
      angular_speed = angular_speed_value;
      dirty = true;
      break;
    // 右转
    case teleop::KEYCODE_RIGHT_ANGULAR:
      ROS_DEBUG("RIGHT");
      angular_speed = -angular_speed_value;
      dirty = true;
      break;
    // 前进
    case teleop::KEYCODE_UP_LINEAR_X:
      ROS_DEBUG("UP");
      linear_speed_x = linear_speed_x_value;
      dirty = true;
      break;
    // 后退
    case teleop::KEYCODE_DOWN_LINEAR_X:
      ROS_DEBUG("DOWN");
      linear_speed_x = -linear_speed_x_value;
      dirty = true;
      break;
    // 左平移
    case teleop::KEYCODE_A_LINEAR_Y_LEFT:
      linear_speed_y = linear_speed_y_value;
      dirty = true;
      break;
    // 右平移
    case teleop::KEYCODE_D_LINEAR_Y_RIGHT:
      linear_speed_y = -linear_speed_y_value;
      dirty = true;
      break;
    }

    // 转换数据类型为geometry_msgs，并发布类型消息
    geometry_msgs::Twist twist;
    twist.angular.z = angular_speed;
    twist.linear.x  = linear_speed_x;
    twist.linear.y  = linear_speed_y;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);
      dirty=false;
    }
  }

  return;
}
