#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"
#include "gimbal_task.h"
#include "CAN_receive.h"
#include "bmi088_task.h"
#include "chassis_behaviour.h"
#include "gimbal_behaviour.h"
#include "remote_control.h"
#include "Vofa.h"
#include "bmi088_task.h"


gimbal_control_t gimbal_control;
static const fp32 yaw_omega_filter_num[1] = {0.2f};
//用于记录开机时云台电机的机械零点
fp32 yaw_motor_offset = 0.0f;

void gimbal_motor_init(gimbal_control_t *gimbal_motor_init);
void gimbal_motor_control(gimbal_control_t *gimbal_motor_control);

void gimbal_task(void const * argument)
{
    gimbal_motor_init(&gimbal_control); //初始化
    CAN_cmd_dm_j4310_mit_enable();  //使能达妙电机
    CAN_cmd_dm_j4310_mit(0, 0, 0, 0, 0);
    vTaskDelay(500);
    CAN_cmd_dm_j4310_mit_zero_cmd();
    vTaskDelay(200);
    while(1)
    {
        gimbal_motor_control(&gimbal_control);
        vTaskDelay(2);
    }
}

/**
  * @brief          指针初始化
  * @param[out]     none
  * @retval         none
  */
void gimbal_motor_init(gimbal_control_t *gimbal_motor_init)
{
    if (gimbal_motor_init == NULL)
    {
        return;
    }

    gimbal_motor_init->gimbal_RC = get_remote_control_point();
    //获取双板通信数据指针
    gimbal_motor_init->board_measure = get_boards_measure_point();
    //获取达妙电机数据指针
    gimbal_motor_init->motor_measure = get_DM_motor_measure_point();
    //上电时记录电机的绝对位置作为0点偏移
    yaw_motor_offset = gimbal_motor_init->motor_measure->position;

    // 初始化位置环PID
    gimbal_PID_init(&gimbal_motor_init->yaw_rotation_pid, YAW_ROT_PID_MAX_OUT, YAW_ROT_PID_MAX_IOUT, YAW_ROT_PID_KP, YAW_ROT_PID_KI, YAW_ROT_PID_KD);
    gimbal_PID_init(&gimbal_motor_init->yaw_follow_chassis_pid, YAW_FOLLOW_PID_MAX_OUT, YAW_FOLLOW_PID_MAX_IOUT, YAW_FOLLOW_PID_KP, YAW_FOLLOW_PID_KI, YAW_FOLLOW_PID_KD);
    gimbal_motor_init->yaw_rotation_omega_set = 0.0f;
    gimbal_motor_init->yaw_follow_omega_set = 0.0f;

    //一阶低通滤波初始化
    //first_order_filter_init(&gimbal_motor_init->yaw_omega_filter, 0.005f, yaw_omega_filter_num);
}

/**
  * @brief          发送控制
  * @param[out]     none
  * @retval         none
  */
void gimbal_motor_control(gimbal_control_t *gimbal_motor_control)
{
    if (gimbal_motor_control == NULL)
    {
        return;
    }
    if (chassis_behaviour_mode == CHASSIS_ROTATION)     //小陀螺模式
    {
        gimbal_rotation_control(gimbal_motor_control);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        gimbal_no_follow_chassis_control(gimbal_motor_control);   //底盘不跟随云台
    }
    else if (chassis_behaviour_mode == CHASSIS_FOLLOW_YAW)
    {
        gimbal_follow_chassis_control(gimbal_motor_control);    //底盘跟随云台
    }
}

/**
  * @brief          云台角度PID初始化, 因为角度范围在(-pi,pi)，不能用PID.c的PID
  * @param[out]     pid:云台PID指针
  * @param[in]      maxout: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

/**
  * @brief          云台角度PID计算, 因为角度范围在(-pi,pi)，不能用PID.c的PID
  * @param[out]     pid:云台PID指针
  * @param[in]      get: 角度反馈
  * @param[in]      set: 角度设定
  * @param[in]      error_delta: 角速度
  * @retval         pid 输出
  */
fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}
