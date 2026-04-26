#ifndef GIMBAL_REMOTE_GIMBAL_TASK_H
#define GIMBAL_REMOTE_GIMBAL_TASK_H
#include "remote_control.h"
#include "CAN_receive.h"
#include "user_lib.h"

// 小陀螺参数 输出单位为rad/s
#define YAW_ROT_PID_KP  13.0f
#define YAW_ROT_PID_KI  0.01f
#define YAW_ROT_PID_KD  0.02f
#define YAW_ROT_PID_MAX_OUT  30.0f  // 限制最大输出速度，对应达妙VMAX
#define YAW_ROT_PID_MAX_IOUT 0.0f

// 底盘跟随云台参数 输出单位为rad/s
#define YAW_FOLLOW_PID_KP  25.0f
#define YAW_FOLLOW_PID_KI  0.01f
#define YAW_FOLLOW_PID_KD  0.01f
#define YAW_FOLLOW_PID_MAX_OUT  30.0f  // 限制最大输出速度，对应达妙VMAX
#define YAW_FOLLOW_PID_MAX_IOUT 0.0f

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{
    const RC_ctrl_t *gimbal_RC;             //获取遥控器指针
    const motor_measure_DM_t *motor_measure;//获取达妙电机数据
    const boards_measure_t *board_measure;  //获取云台C板的数据

    gimbal_PID_t yaw_rotation_pid;         //小陀螺pid
    gimbal_PID_t yaw_follow_chassis_pid;    //底盘跟随云台pid
    fp32 yaw_rotation_omega_set;            //小陀螺位置环输出
    fp32 yaw_follow_omega_set;              //底盘跟随云台位置环输出
    first_order_filter_type_t yaw_omega_filter;
}gimbal_control_t;

extern gimbal_control_t gimbal_control;
extern fp32 yaw_motor_offset;
void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

#endif //GIMBAL_REMOTE_GIMBAL_TASK_H