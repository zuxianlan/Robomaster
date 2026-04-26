#include "FreeRTOS.h"
#include "task.h"
#include "gimbal_task.h"
#include "CAN_receive.h"
#include "bmi088_task.h"
#include "remote_control.h"
#include "Vofa.h"

gimbal_control_t gimbal_control;

void gimbal_motor_init(gimbal_control_t *gimbal_motor_init);
void gimbal_motor_control(gimbal_control_t *gimbal_motor_control);

void gimbal_task(void const * argument)
{
    gimbal_motor_init(&gimbal_control); //初始化
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);
    vTaskDelay(100);
    while(1)
    {
        gimbal_motor_control(&gimbal_control);
        vTaskDelay(10);
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
    //获取双板通信数据指针
    //gimbal_motor_init->two_board_measure = get_two_boards_measure_point();

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
    CAN_cmd_boards(euler_angle[2], deadzone_gyro_z, real_gyro[0]);
}