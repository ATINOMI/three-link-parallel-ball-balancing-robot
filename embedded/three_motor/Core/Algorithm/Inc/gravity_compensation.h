#ifndef GRAVITY_COMPENSATION_H_
#define GRAVITY_COMPENSATION_H_

#include <stdint.h>

// theta_rad：主动关节几何角，水平朝外为零；本次上电标定为12.77度。
// 单位 rad，上升时角度减小；输入来自已加标定偏置的编码器角度。
// pose：正解迭代初值及输出 [psi_x, psi_y, Zp]，单位 rad、rad、m。
// 首次给 pose=[0,0,0.09]，随后保留上次结果，以跟踪平台在底座上方的装配分支。
// k_g：重力补偿比例；torque_nm：缩放后的命令力矩，单位 N*m。
// current_cmd：由命令力矩换算的 C620 电流指令。
// 输入为正常工作空间内的非奇异位形；平台质心位于原点。
void GravityCompensation(const float theta_rad[3], float pose[3], float k_g,
                         float torque_nm[3], int16_t current_cmd[3]);

#endif  // GRAVITY_COMPENSATION_H_
