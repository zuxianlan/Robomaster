#ifndef CHASSIS_REMOTE_2_GIMBAL_BEHAVIOUR_H
#define CHASSIS_REMOTE_2_GIMBAL_BEHAVIOUR_H

#include "gimbal_behaviour.h"
#include "gimbal_task.h"
/**
 * @brief 普通不跟随模式控制函数
 */
void gimbal_no_follow_chassis_control(gimbal_control_t *gimbal_no_follow_chassis);

/**
 * @brief 普通跟随云台控制函数
 */
void gimbal_follow_chassis_control(gimbal_control_t *gimbal_no_follow_chassis);

/**
 * @brief 底盘小陀螺(ROTATION)行为控制函数
 * @note  云台保持不动
 */
void gimbal_rotation_control(gimbal_control_t *gimbal_rotation_control);

#endif //CHASSIS_REMOTE_2_GIMBAL_BEHAVIOUR_H