#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"
#include "gimbal_task.h"
#include "CAN_receive.h"
#include "bmi088_task.h"
#include "remote_control.h"
#include "Vofa.h"
#include "bmi088_task.h"
#include "gimbal_behaviour.h"
#include "chassis_behaviour.h"

/**
 * @brief 底盘不跟随云台控制函数
 */
void gimbal_no_follow_chassis_control(gimbal_control_t *gimbal_no_follow_chassis)
{
    if (gimbal_no_follow_chassis == NULL) return;
    int16_t ch0_set = -gimbal_no_follow_chassis->board_measure->ch0 / 66;

    // float data[1];
    // data[0] = ch1_set;
    // VOFA_Transmit_JustFloat(data, 1);

    CAN_cmd_dm_j4310_mit(0.0f, ch0_set, 0.0f, 0, 0.5f);
}

/**
 * @brief 底盘跟随云台控制函数
 */
void gimbal_follow_chassis_control(gimbal_control_t *gimbal_follow_chassis)
{
    if (gimbal_follow_chassis == NULL) return;
    float ch1_set = -(float)(gimbal_follow_chassis->board_measure->ch0) / 660.0f * PI;

    CAN_cmd_dm_j4310_mit(ch1_set, 0, 0.0f, 2.0f, 0.2f);

    //云台IMU的yaw轴,pitch轴,gyro_z,gyro_x
    fp32 yaw_angle_get = gimbal_follow_chassis->motor_measure->position- yaw_motor_offset;
    fp32 gyro_z_get = gimbal_follow_chassis->board_measure->gyro_z;
    if (fabs(yaw_angle_get) < 0.2f) yaw_angle_get = 0.0f;

    float data[5];
    data[0] = gimbal_follow_chassis->board_measure->yaw;
    data[1] = gyro_z_get;
    data[2] = yaw_angle_get;
    VOFA_Transmit_JustFloat(data, 3);
    //位置环计算
    gimbal_follow_chassis->yaw_follow_omega_set = gimbal_PID_calc(&gimbal_follow_chassis->yaw_follow_chassis_pid, yaw_angle_get, 0.0f, gyro_z_get);

}


/**
 * @brief 底盘小陀螺(ROTATION)行为控制函数
 * @note  云台保持不动
 */
void gimbal_rotation_control(gimbal_control_t *gimbal_rotation_control)
{
    if (gimbal_rotation_control == NULL) return;

    //云台IMU的yaw轴,pitch轴,gyro_z,gyro_x
    fp32 yaw_angle_get = gimbal_rotation_control->board_measure->yaw;
    //fp32 pitch_angle_get = gimbal_rotation_control->board_measure->pitch;
    fp32 gyro_z_get = gimbal_rotation_control->board_measure->gyro_z;
    //fp32 gyro_x_get = gimbal_rotation_control->board_measure->gyro_x;

    //获得真实的yaw轴角速度
    //fp32 yaw_omega_get = arm_cos_f32(pitch_angle_get) * gyro_z_get - arm_sin_f32(pitch_angle_get) * gyro_x_get;
    // fp32 yaw_omega_get = gyro_z_get;
    // first_order_filter_cali(&gimbal_rotation_control->yaw_omega_filter, yaw_omega_get);
    // fp32 yaw_omega_get_filter = gimbal_rotation_control->yaw_omega_filter.out;

    // float data[5];
    // data[0] = gyro_z_get;
    // data[2] = yaw_angle_get;
    //VOFA_Transmit_JustFloat(data, 4);

    //位置环计算
    gimbal_rotation_control->yaw_rotation_omega_set = gimbal_PID_calc(&gimbal_rotation_control->yaw_rotation_pid, yaw_angle_get, 0.0f, gyro_z_get);
    //速度环计算
    CAN_cmd_dm_j4310_mit(0, gimbal_rotation_control->yaw_rotation_omega_set, 0, 0, 0.23);
}