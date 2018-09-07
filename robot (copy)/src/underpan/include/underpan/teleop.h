#ifndef TELEOP_H
#define TELEOP_H
namespace teleop {

/**
 * @brief 声明控制按键的十六进制编码常量
 */
const unsigned char KEYCODE_RIGHT_ANGULAR    = 0x43;      // 控制顺时针角速度
const unsigned char KEYCODE_LEFT_ANGULAR     = 0x44;      // 控制逆时针角速度
const unsigned char KEYCODE_UP_LINEAR_X      = 0x41;      // 控制X轴前进线速度
const unsigned char KEYCODE_DOWN_LINEAR_X    = 0x42;      // 控制X轴后退线速度
const unsigned char KEYCODE_A_LINEAR_Y_LEFT  = 0x61;      // 控制Y轴向左线速度
const unsigned char KEYCODE_D_LINEAR_Y_RIGHT = 0x64;      // 控制Y轴向右线速度
const unsigned char KEYCODE_W_ACCELERATE     = 0x77;      // 加速
const unsigned char KEYCODE_S_DECELERATE     = 0x73;      // 减速
const unsigned char KEYCODE_SPACE_STOP       = 0x00;      // 控制刹车
const unsigned char KEYCODE_Q                = 0x71;


}

#endif // TELEOP_H
