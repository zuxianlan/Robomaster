#ifndef CHASSIS_REMOTE_CHASSIS_TASK_H
#define CHASSIS_REMOTE_CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "stm32f4xx_hal.h"

#define FILTER 0.4f
#define M3505_MOTOR_SPEED_PID_KP 8000.0f  //15
#define M3505_MOTOR_SPEED_PID_KI 2.0f   //0.2
#define M3505_MOTOR_SPEED_PID_KD 10.0f   //5.0
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 5000.0f

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.0004158097489f
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
//摇杆死区
#define CHASSIS_RC_DEADLINE 20
//半轴距
#define MOTOR_DISTANCE_TO_CENTER 0.44f  //0.20（宽） + 0.24（长）
//通道值   s[0]
#define CHASSIS_MODE_CHANNE1 0
#define CHASSIS_MODE_CHANNE2 1

typedef struct
{
    const motor_measure_DJI_t *motor_measure_DJI;
    int16_t rpm;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;//发送的值
    int16_t current;//返回的电流值

} chassis_motor_t;

typedef struct
{
    const boards_measure_t *board_measure;         //获取云台C板的数据

    chassis_motor_t motor_chassis_measure[4];      //底盘电机数据
    pid_type_def chassis_motor_pid[4];             //底盘移动电机pid

    fp32 vx;                          //底盘速度 前进方向 前为正，单位 m/s
    fp32 wz;                          //底盘旋转角速度 单位 rad/s
    fp32 vx_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
    fp32 vy_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
    fp32 wz_set;                      //设定旋转速度, 单位 rad/s

    gimbal_control_t *gimbal;

} chassis_move_t;

extern chassis_move_t chassis_move;
void chassis_task(void *argument);

#endif //CHASSIS_REMOTE_CHASSIS_TASK_H
