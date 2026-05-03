#ifndef BIM088_BMI088_TASK_H
#define BIM088_BMI088_TASK_H
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

#define FILTER_RATIO 0.5f
#define signf(x) ((x) > 0.0f ? 1.0f : ((x) < 0.0f ? -1.0f : 0.0f))

void bmi088_task(void const * argument);
void IMU_KF_Init(void);

extern fp32 euler_angle[3]; //滤波后的欧拉角
extern fp32 real_gyro[3];   //真实的角速度
extern fp32 deadzone_gyro_z;
extern fp32 yaw;

#endif //BIM088_BMI088_TASK_H