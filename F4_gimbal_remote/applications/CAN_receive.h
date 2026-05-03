/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2
#define PMAX 12.56636f
#define VMAX 30.0f
#define TMAX 10.0f
/* CAN send and receive ID */
typedef enum
{
    //DJI 3508电机ID
    CAN_CHASSIS_ALL_ID = 0x200,    //控制报文标识符
    MOTOR_1_ID = 0x201,
	MOTOR_2_ID = 0x202,
	MOTOR_3_ID = 0x203,
	MOTOR_4_ID = 0x204,

	//DM 达妙电机ID
	GIMBAL_ID = 0x01,
	MOTOR_5_ID = 0x12,

	//双板通信ID
	IMU_ID = 0x300,
	REMOTE_ID_CH = 0x302,
	REMOTE_ID_S = 0x303,
} can_msg_id_e;

//大疆电机反馈数据
typedef struct
{	
	uint16_t ecd;     //编码器数据
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
} motor_measure_DJI_t;

/**
 * @brief 发送板子IMU数据 (浮点数，分两帧发送)
 * @param[in] yaw: 俯仰角
 * @param[in] gyro_z: Z轴角速度
 * @param[in] gyro_x: X轴角速度
 * @retval none
 */
void CAN_cmd_boards(float yaw, float gyro_z);

/**
 * @brief 发送遥控器数据
 * @param[in] ch0: 通道0
 * @param[in] ch1: 通道1
 * @param[in] ch2: 通道2
 * @param[in] ch3: 通道3
 * @retval none
 */
void CAN_cmd_remote_ch(int16_t ch0, int16_t ch1, int16_t ch2, int16_t ch3);

/**
 * @brief 发送遥控器数据
 * @param[in] s0: 右拨杆
 * @param[in] s1: 左拨杆
 * @retval none
 */
void CAN_cmd_remote_s(int16_t s0, int16_t s1);

#endif
