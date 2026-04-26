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

// 达妙电机反馈数据
typedef struct {
	uint8_t  id;			// 电机ID
	uint8_t  error;         // 电机状态
	fp32  position;      // 位置
	fp32  omega;         // 角速度
	fp32  torque;	    // 扭矩
	fp32  temp_mos;      // MOS温度
	fp32  temp_rotor;    // 转子温度
} motor_measure_DM_t;

extern motor_measure_DM_t motor_DM;

/**
  * @brief          发送电机控制电流
  * @param[in]      motor_3508: 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor_3508_1, int16_t motor_3508_2, int16_t motor_3508_3, int16_t motor_3508_4);

/**
 * @brief 发送达妙电机MIT模式控制电流 (修正位拼接逻辑)
 */
void CAN_cmd_dm_j4310_mit(float angle, float omega, float torque, float kp, float kd);

/**
 * @brief 发送达妙电机使能
 */
void CAN_cmd_dm_j4310_mit_enable(void);

/**
 * @brief 发送达妙电机保存位置零点
 */
void CAN_cmd_dm_j4310_mit_zero_cmd(void);

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_DJI_t *get_chassis_motor_measure_point(uint8_t i);

/**
  * @brief          返回云台电机 达妙电机数据指针
  * @param
  * @retval         电机数据指针
  */
extern const motor_measure_DM_t *get_DM_motor_measure_point(void);

#endif
