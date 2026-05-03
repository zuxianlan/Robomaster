#include "FreeRTOS.h"
#include "task.h"
#include "gimbal_task.h"
#include "CAN_receive.h"
#include "bmi088_task.h"
#include "remote_control.h"
#include "detect.h"
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
        detect_rc_tick();
        if (is_rc_off())
        {
            CAN_cmd_boards(0, 0);
            CAN_cmd_remote_s(2, 2);
            CAN_cmd_remote_ch(0, 0, 0, 0);
        }
        else
        {
            gimbal_motor_control(&gimbal_control);
        }
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
    int16_t ch0 = gimbal_motor_control->gimbal_RC->rc.ch[0];
    int16_t ch1 = gimbal_motor_control->gimbal_RC->rc.ch[1];
    int16_t ch2 = gimbal_motor_control->gimbal_RC->rc.ch[2];
    int16_t ch3 = gimbal_motor_control->gimbal_RC->rc.ch[3];
    int16_t s0 = gimbal_motor_control->gimbal_RC->rc.s[0];
    int16_t s1 = gimbal_motor_control->gimbal_RC->rc.s[1];

    CAN_cmd_boards(yaw, deadzone_gyro_z);
    CAN_cmd_remote_ch(ch0, ch1, ch2, ch3);
    CAN_cmd_remote_s(s0, s1);
}