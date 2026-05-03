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
#include "gimbal_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CAN_TxHeaderTypeDef boards_tx_message;
static uint8_t			   boards_can_send_data[8];


/**
 * @brief 发送板子IMU数据 (浮点数，分两帧发送)
 * @param[in] raw_yaw: 偏航角
 * @param[in] raw_gyro_z: Z轴角速度
 * @retval none
 */
void CAN_cmd_boards(float raw_yaw, float raw_gyro_z)
{
	uint32_t send_mail_box;

	int16_t yaw = (int16_t)(raw_yaw * 10000);
	int16_t gyro_z = (int16_t)(raw_gyro_z * 10000);
	boards_tx_message.StdId = IMU_ID;
	boards_tx_message.IDE = CAN_ID_STD;
	boards_tx_message.RTR = CAN_RTR_DATA;
	boards_tx_message.DLC = 0x04;
	boards_can_send_data[0] = yaw >> 8;
	boards_can_send_data[1] = yaw;
	boards_can_send_data[2] = gyro_z >> 8;
	boards_can_send_data[3] = gyro_z;
	HAL_CAN_AddTxMessage(&hcan2, &boards_tx_message, boards_can_send_data, &send_mail_box);
}

/**
 * @brief 发送遥控器数据
 * @param[in] ch0: 通道0
 * @param[in] ch1: 通道1
 * @param[in] ch2: 通道2
 * @param[in] ch3: 通道3
 * @retval none
 */
void CAN_cmd_remote_ch(int16_t ch0, int16_t ch1, int16_t ch2, int16_t ch3)
{
	uint32_t send_mail_box;
	boards_tx_message.StdId = REMOTE_ID_CH;
	boards_tx_message.IDE = CAN_ID_STD;
	boards_tx_message.RTR = CAN_RTR_DATA;
	boards_tx_message.DLC = 0x08;
	boards_can_send_data[0] = ch0 >> 8;
	boards_can_send_data[1] = ch0;
	boards_can_send_data[2] = ch1 >> 8;
	boards_can_send_data[3] = ch1;
	boards_can_send_data[4] = ch2 >> 8;
	boards_can_send_data[5] = ch2;
	boards_can_send_data[6] = ch3 >> 8;
	boards_can_send_data[7] = ch3;
	HAL_CAN_AddTxMessage(&hcan2, &boards_tx_message, boards_can_send_data, &send_mail_box);
	// if (HAL_CAN_AddTxMessage(&hcan2, &boards_tx_message, boards_can_send_data, &send_mail_box) != HAL_OK)
	// {
	// 	HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
	// }
}

/**
 * @brief 发送遥控器数据
 * @param[in] s0: 右拨杆
 * @param[in] s1: 左拨杆
 * @retval none
 */
void CAN_cmd_remote_s(int16_t s0, int16_t s1)
{
	uint32_t send_mail_box;
	boards_tx_message.StdId = REMOTE_ID_S;
	boards_tx_message.IDE = CAN_ID_STD;
	boards_tx_message.RTR = CAN_RTR_DATA;
	boards_tx_message.DLC = 0x04;
	boards_can_send_data[0] = s0 >> 8;
	boards_can_send_data[1] = s0;
	boards_can_send_data[2] = s1 >> 8;
	boards_can_send_data[3] = s1;
	HAL_CAN_AddTxMessage(&hcan2, &boards_tx_message, boards_can_send_data, &send_mail_box);
	// if (HAL_CAN_AddTxMessage(&hcan2, &boards_tx_message, boards_can_send_data, &send_mail_box) != HAL_OK)
	// {
	// 	HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
	// }
}