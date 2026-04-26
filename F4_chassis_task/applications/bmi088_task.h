#ifndef BIM088_BMI088_TASK_H
#define BIM088_BMI088_TASK_H
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

void bmi088_task(void const * argument);
void IMU_KF_Init(void);

typedef struct
{
    fp32 pitch;
    fp32 roll;
    fp32 yaw;
    fp32 omega;
}euler_angle_t;

extern euler_angle_t euler_angle;
#endif //BIM088_BMI088_TASK_H