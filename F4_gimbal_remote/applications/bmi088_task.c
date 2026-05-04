#include "bmi088_task.h"
#include "BMI088driver.h"
#include "cmsis_os.h"
#include "spi.h"
#include "usart.h"
#include "task.h"
#include "Vofa.h"
#include "math.h"
#include "kalman_filter.h"

fp32 gyro[3], accel[3], real_gyro[3];
fp32 deadzone_gyro_z = 0.0f;
fp32 dt = 0.002f; // 任务周期 2ms

// 3个零偏估计器 (X, Y, Z轴)
KalmanFilter_t Gyro_Bias_KF[3];
fp32 gyro_bias[3] = {0, 0, 0};

// 2个姿态融合器 (Pitch, Roll)
KalmanFilter_t Attitude_KF[2];
fp32 euler_angle[3] = {0, 0, 0}; // [0]:Pitch, [1]:Roll, [2]:Yaw
fp32 yaw = 0.0f;
void bmi088_task(void const * argument)
{
    while(BMI088_init()) {;}
    IMU_KF_Init();

    while (1)
    {
        BMI088_read(gyro, accel);

        float acc_mag = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
        uint8_t is_acc_static = (fabs(acc_mag - 9.8f) < 1.6f) ? 1 : 0;

        // X轴(Pitch)静止条件：加速度正常 且 X轴陀螺仪不动
        uint8_t is_static_x = (is_acc_static && fabs(gyro[0]) < 0.3f) ? 1 : 0;
        // Y轴(Roll)静止条件：加速度正常 且 Y轴陀螺仪不动
        uint8_t is_static_y = (is_acc_static && fabs(gyro[1]) < 0.3f) ? 1 : 0;
        // Z轴(Yaw)静止条件：加速度正常 且 Z轴陀螺仪不动
        uint8_t is_static_z = (is_acc_static && fabs(gyro[2]) < 0.3f) ? 1 : 0;

        uint8_t static_flags[3] = {is_static_x, is_static_y, is_static_z};

        // 更新X/Y/Z三个轴的陀螺仪零偏
        for(int i = 0; i < 3; i++)
        {
            if (static_flags[i])
            {
                Gyro_Bias_KF[i].MeasuredVector[0] = gyro[i];
            }
            else
            {
                Gyro_Bias_KF[i].MeasuredVector[0] = 0;
            }
            Kalman_Filter_Update(&Gyro_Bias_KF[i]);
            gyro_bias[i] = Gyro_Bias_KF[i].FilteredValue[0];
        }
        // 计算去除零偏后的“真实”角速度
        for(int i = 0; i < 3; i++) real_gyro[i] = gyro[i] - gyro_bias[i];

        // 计算加速度计观测的Pitch和Roll
        fp32 pitch_acc = atan2(-accel[0], accel[2]);
        fp32 roll_acc  = atan2( accel[1], accel[2]);

        // 融合Pitch和Roll (卡尔曼互补滤波)
        // 喂入控制量：陀螺仪积分增量
        Attitude_KF[0].ControlVector[0] = real_gyro[0] * dt; // X轴角速度对应Pitch
        Attitude_KF[1].ControlVector[0] = real_gyro[1] * dt; // Y轴角速度对应Roll
        // 喂入观测量：加速度计算出的角度
        Attitude_KF[0].MeasuredVector[0] = pitch_acc;
        Attitude_KF[1].MeasuredVector[0] = roll_acc;
        // 注意：姿态KF不需要Skip，无论动与静，都要让陀螺仪和加速度计互相牵制！
        Kalman_Filter_Update(&Attitude_KF[0]);
        Kalman_Filter_Update(&Attitude_KF[1]);

        euler_angle[0] = Attitude_KF[0].FilteredValue[0]; // 平滑的Pitch
        euler_angle[1] = Attitude_KF[1].FilteredValue[0]; // 平滑的Roll


        //积分计算Yaw (加入低通滤波和死区处理)
        static fp32 lpf_gyro_z = 0.0f; // 静态变量保存上一时刻的滤波值
        //一阶低通滤波 (平滑电机带来的高频震动)
        // 参数说明：0.85 决定平滑程度，越大越平滑，但转弯响应越慢。建议范围 0.8~0.95
        lpf_gyro_z = lpf_gyro_z * FILTER_RATIO + real_gyro[2] * (1 - FILTER_RATIO);

        //死区限制 (切除底盘直行时的底噪漂移)
        deadzone_gyro_z = lpf_gyro_z;
        float yaw_deadzone = 0.03f; //死区阈值，单位 rad/s
        if (fabs(deadzone_gyro_z) <= yaw_deadzone) {
            // 在死区内，认为是噪声，强制为0
            deadzone_gyro_z = 0.0f;
        } else {
            // 超出死区的部分，要减去死区的宽度，实现平滑衔接
            // signf 是取符号函数，如果是正数就是+1，负数就是-1
            deadzone_gyro_z = deadzone_gyro_z - signf(deadzone_gyro_z) * yaw_deadzone;
        }

        //用处理后的角速度进行积分
        euler_angle[2] += deadzone_gyro_z * dt;
        yaw = euler_angle[2];

        float data[6];
        data[0] = real_gyro[2];
        data[1] = lpf_gyro_z;
        data[2] = euler_angle[2];     //Yaw角

        data[3] = deadzone_gyro_z;
        data[4] = euler_angle[1];     // 通道3: 滤波后Roll


        VOFA_Transmit_JustFloat(data, 6);
        vTaskDelay(2);
    }
}

//初始化所有的卡尔曼滤波器
void IMU_KF_Init(void)
{
    // --- 零偏KF所需数组 ---
    static float F_Bias[1] = {1.0f};
    static float Q_Bias[1] = {0.00001f};
    static float P_Bias[1] = {1.0f};
    static float min_var_bias[1] = {0.0005f};
    // 因为开启了自动调整，需要配置H和R的生成依据
    static uint8_t map_bias[1] = {1};       // 第1个观测量对应第1个状态
    static float deg_bias[1] = {1.0f};      // H矩阵系数
    static float r_diag_bias[1] = {0.005f}; // R对角线元素

    for(int i = 0; i < 3; i++) {
        Kalman_Filter_Init(&Gyro_Bias_KF[i], 1, 0, 1);
        // 【修改点5】：开启自动调整，让MeasuredVector=0时自动退化为纯时间推移
        Gyro_Bias_KF[i].UseAutoAdjustment = 1;

        memcpy(Gyro_Bias_KF[i].F_data, F_Bias, sizeof(F_Bias));
        memcpy(Gyro_Bias_KF[i].Q_data, Q_Bias, sizeof(Q_Bias));
        memcpy(Gyro_Bias_KF[i].P_data, P_Bias, sizeof(P_Bias));
        memcpy(Gyro_Bias_KF[i].StateMinVariance, min_var_bias, sizeof(min_var_bias));
        // 配置自动调整所需的映射和方差
        memcpy(Gyro_Bias_KF[i].MeasurementMap, map_bias, sizeof(map_bias));
        memcpy(Gyro_Bias_KF[i].MeasurementDegree, deg_bias, sizeof(deg_bias));
        memcpy(Gyro_Bias_KF[i].MatR_DiagonalElements, r_diag_bias, sizeof(r_diag_bias));
    }

    // --- 姿态KF所需数组 ---
    static float F_Att[1] = {1.0f};
    static float B_Att[1] = {0.01f};
    static float H_Att[1] = {1.0f};
    static float Q_Att[1] = {0.004f};
    static float R_Att[1] = {0.5f};
    static float P_Att[1] = {1.0f};
    static float min_var_att[1] = {0.0001f};

    for(int i = 0; i < 2; i++) {
        Kalman_Filter_Init(&Attitude_KF[i], 1, 1, 1);
        Attitude_KF[i].UseAutoAdjustment = 0; // 姿态KF不需要自动调整，始终有加速度计数据

        memcpy(Attitude_KF[i].F_data, F_Att, sizeof(F_Att));
        memcpy(Attitude_KF[i].B_data, B_Att, sizeof(B_Att));
        memcpy(Attitude_KF[i].H_data, H_Att, sizeof(H_Att));
        memcpy(Attitude_KF[i].Q_data, Q_Att, sizeof(Q_Att));
        memcpy(Attitude_KF[i].R_data, R_Att, sizeof(R_Att));
        memcpy(Attitude_KF[i].P_data, P_Att, sizeof(P_Att));
        memcpy(Attitude_KF[i].StateMinVariance, min_var_att, sizeof(min_var_att));
    }
}
