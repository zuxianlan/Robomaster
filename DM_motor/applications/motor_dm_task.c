#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "motor_dm_task.h"
#include "CAN_receive.h"
#include "Vofa.h"
#include "usart.h"

uint32_t Counter = 0;
float target_speed = 0.0f;

void motor_dm_task(void const * argument)
{
    //获取中断的电机数据指针
    //const motor_measure_DM *motor_DM_j4310 = get_DM_motor_measure_point();
    //使能电机
    CAN_cmd_dm_j4310_mit_enable();
    vTaskDelay(100);
    // CAN_cmd_dm_j4310_mit(0.0f, 0.0f, 0.0f, 8.0f, 0.5f);
    // vTaskDelay(100);
    //HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);

    //保存位置零点
    CAN_cmd_dm_j4310_mit_zero_cmd();
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
    vTaskDelay(1000);
    while (1)
    {
        //Counter++;
        //在 0 和 PI 之间来回摆动
        //target_speed = (Counter / 3000) % 2 == 0 ? 2.0f : 5.0f;

        // 参数：目标角度，目标速度(0表示纯位置控制)，前馈扭矩(0)，Kp，Kd
        CAN_cmd_dm_j4310_mit(0, 1.0f, 0.0f, 0.0f, 0.7f);
        float angle = motor_DM.position;
        while (angle > PI)  angle -= 2.0f * PI;
        while (angle < -PI) angle += 2.0f * PI;

        float data[4];
        data[0] = angle;
        data[1] = motor_DM.omega;
        data[2] = motor_DM.torque;
        data[3] = motor_DM.id;
        VOFA_Transmit_JustFloat(data, 4);

        vTaskDelay(2);
    }
}