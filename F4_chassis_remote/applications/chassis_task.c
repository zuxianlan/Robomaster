#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "remote_control.h"
#include "pid.h"
#include "CAN_receive.h"
#include "Vofa.h"
#include "detect.h"
#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include <math.h>
#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "arm_math.h"


fp32 M3508_current;

chassis_move_t chassis_move;

static void chassis_set_contorl(chassis_move_t *chassis_move_control);
void chassis_move_init(chassis_move_t *chassis_move_init);
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

void chassis_task(void *argument)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
    chassis_move_init(&chassis_move);
    //CAN_cmd_dm_j4310_mit_enable();  //使能达妙电机
    //CAN_cmd_dm_j4310_mit_zero_cmd();
    vTaskDelay(300);
    while (1)
    {
        if (switch_is_down(chassis_move.board_measure->s1))
        {
            CAN_cmd_dm_j4310_mit_disable();
            CAN_cmd_chassis(0, 0, 0, 0);
        }
        else
        {
            //设置模式
            chassis_set_contorl(&chassis_move);
            //麦轮解算，pid计算
            chassis_control_loop(&chassis_move);

            //CAN_cmd_dm_j4310_mit_enable();
            CAN_cmd_chassis(-chassis_move.motor_chassis_measure[0].give_current,
                            -chassis_move.motor_chassis_measure[1].give_current,
                            chassis_move.motor_chassis_measure[2].give_current,
                            chassis_move.motor_chassis_measure[3].give_current);
            CAN_cmd_dm_j4310_mit_enable();
        }

        vTaskDelay(2);
    }
}

/**
  * @brief          遥控器初始化，PID初始化
  * @param[out]     none
  * @retval         none
  */
void chassis_move_init(chassis_move_t *chassis_move_init)
{
    //获取遥控器指针，否则会疯转
    //chassis_move_init->chassis_RC = get_remote_control_point();

    chassis_move_init->board_measure = get_boards_measure_point();

    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

    //获取底盘电机数据指针，初始化PID
    for (int i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis_measure[i].motor_measure_DJI = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->chassis_motor_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
}

/**
  * @brief          设置模式
  * @param[out]     none
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }
    fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

    chassis_behaviour_mode_set(chassis_move_control);   //判断并设置当前模式
    chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, chassis_move_control); //根据模式计算 vx, vy, wz 设定值

    //小陀螺模式下的全向移动解算
    if (chassis_behaviour_mode == CHASSIS_ROTATION)
    {
        static fp32 chassis_angle = 0.0f;
        chassis_angle = -gimbal_control.motor_measure->position;

        // float data[1];
        // data[0] = chassis_angle;
        // VOFA_Transmit_JustFloat(data, 1);

        while (chassis_angle > PI) chassis_angle -= 2.0f * PI;
        while (chassis_angle < -PI) chassis_angle += 2.0f * PI;

        //小陀螺全向移动解算
        fp32 sin_yaw = arm_sin_f32(chassis_angle);
        fp32 cos_yaw = arm_cos_f32(chassis_angle);

        fp32 real_vx = cos_yaw * vx_set + sin_yaw * vy_set;
        fp32 real_vy = -sin_yaw * vx_set + cos_yaw * vy_set;
        chassis_move_control->vx_set = real_vx;
        chassis_move_control->vy_set = real_vy;
    }
    else
    {
        chassis_move_control->vx_set = vx_set;     // 非小陀螺模式，直接赋值
        chassis_move_control->vy_set = vy_set;
    }
    chassis_move_control->wz_set = wz_set;
}

/**
  * @brief          四个麦轮速度是通过三个参数计算出来的
  * @param[in]      vx_set: 横向速度
  * @param[in]      vy_set: 纵向速度
  * @param[in]      wz_set: 旋转速度
  * @param[out]     wheel_speed: 四个麦轮速度
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    wheel_speed[0] = -vx_set + vy_set - wz_set * MOTOR_DISTANCE_TO_CENTER;
    wheel_speed[1] = vx_set + vy_set - wz_set * MOTOR_DISTANCE_TO_CENTER;
    wheel_speed[2] = -vx_set + vy_set + wz_set * MOTOR_DISTANCE_TO_CENTER;
    wheel_speed[3] = +vx_set + vy_set + wz_set * MOTOR_DISTANCE_TO_CENTER;

}

/**
  * @brief          麦轮解算，控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    float filter_rpm = 0.0f;
    float filter_pidout = 0.0f;
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set,
                                          chassis_move_control_loop->wz_set,
                                          wheel_speed);
    //计算pid
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis_measure[i].speed_set = wheel_speed[i];

        fp32 raw_rpm = (fp32)(chassis_move_control_loop->motor_chassis_measure[i].motor_measure_DJI->speed_rpm);

        if (i == 0 || i == 1)
        {raw_rpm = -raw_rpm;}
        chassis_move_control_loop->motor_chassis_measure[i].speed = raw_rpm * M3508_MOTOR_RPM_TO_VECTOR;   //转化为m/s

        PID_calc(&chassis_move_control_loop->chassis_motor_pid[i], chassis_move_control_loop->motor_chassis_measure[i].speed, chassis_move_control_loop->motor_chassis_measure[i].speed_set);

        // float data[2];
        // data[0] = chassis_move_control_loop->motor_chassis_measure[0].speed;
        // data[1] = chassis_move_control_loop->motor_chassis_measure[0].speed_set;
        // VOFA_Transmit_JustFloat(data, 2);
    }


    //赋值电流值
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis_measure[i].give_current = (int16_t)(chassis_move_control_loop->chassis_motor_pid[i].out);
    }

}