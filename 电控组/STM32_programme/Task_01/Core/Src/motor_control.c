#include "motor_control.h"

/* 全局变量定义 */
uint8_t dir[4] = {16, 32, 64, 128};

/* 运动信号矩阵：[方向][电机] */
int move_signal[6][4] = {
    {-1, -1, 1, 1},    // 向前运动
    {1, 1, -1, -1},    // 向后运动
    {-1, 1, 1, -1},    // 向左运动
    {1, -1, -1, 1},    // 向右运动
    {-1, 1, -1, 1},    // 左旋
    {1, -1, 1, -1}     // 右旋
};

uint32_t tim_channel[4] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
    TIM_CHANNEL_3,
    TIM_CHANNEL_4
};

/* 定时器映射表：每个电机对应的定时器 */
static TIM_HandleTypeDef* motor_timer_map[MOTOR_COUNT] = {
    &htim4,  // MOTOR_1
    &htim4,  // MOTOR_2
    &htim2,  // MOTOR_3
    &htim2   // MOTOR_4
};

/* 通道映射表：每个电机对应的通道索引 */
static const uint8_t motor_channel_offset[MOTOR_COUNT] = {
    0,  // MOTOR_1 uses CH1/CH2
    2,  // MOTOR_2 uses CH3/CH4
    0,  // MOTOR_3 uses CH1/CH2
    2   // MOTOR_4 uses CH3/CH4
};

/**
 * @brief  电机控制初始化
 * @param  None
 * @retval None
 */
void MotorControl_Init(void)
{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

    HAL_Delay(50);

    // ========== 第二步：立即停止所有电机 ==========
    StopAllMotors();

    HAL_Delay(50);

    // ========== 第三步：启动PWM ==========
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    HAL_Delay(50);

    // ========== 第四步：再次确保停止 ==========
    StopAllMotors();
}


/**
 * @brief  获取移动方向索引
 * @param  ps2_data: PS2手柄数据指针
 * @retval 方向索引 (-1表示停止)
 */
int GetMoveDirection(JOYSTICK_TypeDef *ps2_data)
{
    // 检查模式，如果是模式2则停止
    if(ps2_data->mode == 65 || ps2_data==NULL)
    {
        return MOVE_STOP;
    }

    // 判断方向
    if(ps2_data->LJoy_UD == 0)
    {
        return MOVE_FORWARD;
    }
    else if(ps2_data->LJoy_UD == 255)
    {
        return MOVE_BACKWARD;
    }
    else if(ps2_data->LJoy_LR == 0)
    {
        return MOVE_LEFT;
    }
    else if(ps2_data->LJoy_LR == 255)
    {
        return MOVE_RIGHT;
    }
    else if(ps2_data->btn2 == dir[3])
    {
    	return ROTATE_LEFT;
    }
    else if(ps2_data->btn2 == dir[1])
    {
    	return ROTATE_RIGHT;
    }

    return MOVE_STOP;
}

/**
 * @brief  设置单个电机速度
 * @param  motor_id: 电机ID (0-3)
 * @param  compare1: PWM比较值1
 * @param  compare2: PWM比较值2
 * @retval None
 */
void SetSingleMotor(MotorID_t motor_id, int compare1, int compare2)
{
    if(motor_id >= MOTOR_COUNT)
    {
        return;  // 参数检查
    }

    TIM_HandleTypeDef *timer = motor_timer_map[motor_id];
    uint8_t ch_offset = motor_channel_offset[motor_id];

    __HAL_TIM_SET_COMPARE(timer, tim_channel[ch_offset], compare1);
    __HAL_TIM_SET_COMPARE(timer, tim_channel[ch_offset + 1], compare2);
}

/**
 * @brief  设置电机速度
 * @param  direction_index: 方向索引
 * @param  compare_init_1: PWM比较值1
 * @param  compare_init_2: PWM比较值2
 * @retval None
 */
void SetMotorSpeed(int direction_index, int compare_init_1, int compare_init_2)
{
    // 参数检查
    if(direction_index < 0 || direction_index >= 6)
    {
        return;
    }

    // 遍历所有电机
    for(MotorID_t motor_id = MOTOR_1; motor_id < MOTOR_COUNT; motor_id++)
    {
        int compare1 = compare_init_1;
        int compare2 = compare_init_2;

        // 根据方向信号调整PWM占空比（反向电机）
        if(move_signal[direction_index][motor_id] == -1)
        {
            int temp = compare1;
            compare1 = compare2;
            compare2 = temp;
        }

        // 设置电机PWM
        SetSingleMotor(motor_id, compare1, compare2);
    }
}

/**
 * @brief  停止所有电机
 * @param  None
 * @retval None
 */
void StopAllMotors(void)
{
    // 清零所有TIM4通道
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

    // 清零所有TIM2通道
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}
