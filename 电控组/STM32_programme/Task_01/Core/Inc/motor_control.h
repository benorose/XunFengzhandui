/*
 * motor_control.h
 *
 *  Created on: Oct 15, 2025
 *      Author: 郭安艺
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "tim.h"
#include "ax_ps2.hpp"

/* 运动方向枚举 */
typedef enum {
    MOVE_FORWARD = 0,   // 向前
    MOVE_BACKWARD = 1,  // 向后
    MOVE_LEFT = 2,      // 向左
    MOVE_RIGHT = 3,     // 向右
    ROTATE_LEFT = 4,    // 左旋
    ROTATE_RIGHT = 5,   // 右旋
    MOVE_STOP = -1      // 停止
} MotorDirection_t;

/* 电机编号枚举 */
typedef enum {
    MOTOR_1 = 0,  // 电机1（htim4 CH1/CH2）
    MOTOR_2 = 1,  // 电机2（htim4 CH3/CH4）
    MOTOR_3 = 2,  // 电机3（htim2 CH1/CH2）
    MOTOR_4 = 3,  // 电机4（htim2 CH3/CH4）
    MOTOR_COUNT = 4
} MotorID_t;

/* 函数声明 */
void MotorControl_Init(void);
int GetMoveDirection(JOYSTICK_TypeDef *ps2_data);
void SetMotorSpeed(int direction_index, int compare_init_1, int compare_init_2);
void StopAllMotors(void);
void SetSingleMotor(MotorID_t motor_id, int compare1, int compare2);

/* 外部变量声明 */
extern int move_signal[6][4];
extern uint32_t tim_channel[4];
extern uint8_t dir[4];

#endif /* MOTOR_CONTROL_H */
