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

#include "CAN_receive.h"
#include "main.h"
#include "string.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief 将浮点数转换为无符号整型 (对应 uint_to_float 的逆过程)
 * @param x_float: 输入的浮点物理量
 * @param x_min: 物理量最小值
 * @param x_max: 物理量最大值
 * @param bits: 位宽 (如12位或16位)
 * @retval 转换后的无符号整数
 */
static uint16_t float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	// 限制输入范围，防止溢出
	if (x_float < x_min) x_float = x_min;
	if (x_float > x_max) x_float = x_max;
	return (uint16_t)((x_float - x_min) * ((float)((1 << bits) - 1)) / span);
}

static CAN_TxHeaderTypeDef boards_tx_message;
static uint8_t			   boards_can_send_data[8];


/**
 * @brief 发送板子IMU数据 (浮点数，分两帧发送)
 * @param[in] yaw: 偏航角
 * @param[in] gyro_z: Z轴角速度
 * @param[in] gyro_x: X轴角速度
 * @retval none
 */
void CAN_cmd_boards(float yaw, float gyro_z, float gyro_x)
{
	uint32_t send_mail_box;

	//发送第一帧：yaw
	memcpy(boards_can_send_data, &yaw, 4);
	boards_tx_message.StdId = EULER_ID;
	boards_tx_message.IDE = CAN_ID_STD;
	boards_tx_message.RTR = CAN_RTR_DATA;
	boards_tx_message.DLC = 0x04; // 4个字节
	//HAL_CAN_AddTxMessage(&hcan2, &boards_tx_message, boards_can_send_data, &send_mail_box);
	if (HAL_CAN_AddTxMessage(&hcan2, &boards_tx_message, boards_can_send_data, &send_mail_box) != HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
	}

	//发送第二帧：gyro_z, gyro_x
	memcpy(boards_can_send_data, &gyro_z, 4);      // 前4字节放 gyro_z
	memcpy(boards_can_send_data + 4, &gyro_x, 4);  // 后4字节放 gyro_x
	boards_tx_message.StdId = GYRO_ID;
	boards_tx_message.IDE = CAN_ID_STD;
	boards_tx_message.RTR = CAN_RTR_DATA;
	boards_tx_message.DLC = 0x08;
	//HAL_CAN_AddTxMessage(&hcan2, &boards_tx_message, boards_can_send_data, &send_mail_box);
	if (HAL_CAN_AddTxMessage(&hcan2, &boards_tx_message, boards_can_send_data, &send_mail_box) != HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
	}
}
