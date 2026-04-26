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
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
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

#define get_motor_measure_DJI(ptr, data)								   \
{																		   \
	(ptr)->last_ecd = (ptr)->ecd;                                          \
	(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);                   \
	(ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);              \
	(ptr)->given_current = (int16_t)((data)[4] << 8 | (data)[5]);          \
	(ptr)->temperate = (data)[6];                                          \
}

#define get_motor_measure_DM(ptr, data)                                   \
{                                                                         \
    (ptr)->id = (data)[0] & 0x0F;                                         \
	(ptr)->error = (data)[0] >> 4;										  \
	uint16_t _pos_raw = ((uint16_t)(data)[1] << 8) | (data)[2];           \
	uint16_t _spd_raw = ((uint16_t)(data)[3] << 4) | ((data)[4] >> 4);    \
	uint16_t _trq_raw = (((uint16_t)(data)[4] & 0x0F) << 8) | (data)[5];  \
    (ptr)->temp_mos = (data)[6];										  \
	(ptr)->temp_rotor = (data)[7];                                        \
	(ptr)->position = uint_to_float((int)_pos_raw, -PMAX, PMAX, 16);      \
	(ptr)->omega    = uint_to_float((int)_spd_raw, -VMAX, VMAX, 12);      \
	(ptr)->torque   = uint_to_float((int)_trq_raw, -TMAX, TMAX, 12);      \
}

static motor_measure_DJI_t  motor_chassis[4];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

motor_measure_DM_t         motor_DM;
static CAN_TxHeaderTypeDef dm_tx_message;
static uint8_t			   dm_can_send_data[8];

boards_measure_t           boards_measure;

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t rx_data[8];
	CAN_RxHeaderTypeDef rx_header;

 	if(hcan==&hcan1)
 		{
 		    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
 			switch (rx_header.StdId)
 			{
 				case MOTOR_1_ID:	//M3508电机
 				case MOTOR_2_ID:
 				case MOTOR_3_ID:
 				case MOTOR_4_ID:
 					{
 						static uint8_t i = 0;
 						//get motor id
 						i = rx_header.StdId - MOTOR_1_ID;
 						get_motor_measure_DJI(&motor_chassis[i], rx_data);
 						break;
 					}
 				default:
 						{
 							break;
 						}
 			}
 		}
	else if (hcan==&hcan2)
		{
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
			switch (rx_header.StdId)
			{
				case EULER_ID:
					{
					memcpy(&boards_measure.yaw, rx_data, 4);
					HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
					break;
					}
				case GYRO_ID:
					{
					memcpy(&boards_measure.gyro_z, rx_data, 4);
					memcpy(&boards_measure.gyro_x, rx_data + 4, 4);
					HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
					break;
					}
				case MOTOR_5_ID:
					{
						get_motor_measure_DM(&motor_DM, rx_data);
						//HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
						break;
					}
				default:
					{
						break;
					}
			}
		}
}

/**
  * @brief          发送电机控制电流
  * @param[in]      motor_3508: 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor_3508_1, int16_t motor_3508_2, int16_t motor_3508_3, int16_t motor_3508_4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor_3508_1 >> 8;
    chassis_can_send_data[1] = motor_3508_1;
    chassis_can_send_data[2] = motor_3508_2 >> 8;
    chassis_can_send_data[3] = motor_3508_2;
	chassis_can_send_data[4] = motor_3508_3 >> 8;
	chassis_can_send_data[5] = motor_3508_3;
	chassis_can_send_data[6] = motor_3508_4>> 8;
	chassis_can_send_data[7] = motor_3508_4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief 发送达妙电机MIT模式控制电流
 */
void CAN_cmd_dm_j4310_mit(float angle, float omega, float torque, float kp, float kd)
{
	uint32_t send_mail_box;

	//物理量转为定点数
	uint16_t p_des = float_to_uint(angle, -PMAX, PMAX, 16);  // 位置: 16位
	uint16_t v_des = float_to_uint(omega, -VMAX, VMAX, 12);     // 速度: 12位
	uint16_t t_ff  = float_to_uint(torque, -TMAX, TMAX, 12);   // 扭矩: 12位
	uint16_t kp_raw = float_to_uint(kp, 0.0f, 500.0f, 12);    // Kp: 12位
	uint16_t kd_raw = kd_raw = float_to_uint(kd, 0.0f, 5.0f, 12);     // Kd: 12位

	//进行位拼接
	dm_can_send_data[0] = (uint8_t)(p_des >> 8);                     // p_des[15:8]
	dm_can_send_data[1] = (uint8_t)(p_des);                          // p_des[7:0]
	dm_can_send_data[2] = (uint8_t)(v_des >> 4);                     // v_des[11:4]
	dm_can_send_data[3] = (uint8_t)(((v_des & 0x0F) << 4) | (kp_raw >> 8)); // v_des[3:0]|Kp[11:8]
	dm_can_send_data[4] = (uint8_t)(kp_raw & 0xFF);                  // Kp[7:0]
	dm_can_send_data[5] = (uint8_t)(kd_raw >> 4);                    // Kd[11:4]
	dm_can_send_data[6] = (uint8_t)(((kd_raw & 0x0F) << 4) | (t_ff >> 8)); // Kd[3:0]|t_ff[11:8]
	dm_can_send_data[7] = (uint8_t)(t_ff & 0xFF);                    // t_ff[7:0]

	//发送CAN报文
	dm_tx_message.StdId = GIMBAL_ID;
	dm_tx_message.IDE = CAN_ID_STD;
	dm_tx_message.RTR = CAN_RTR_DATA;
	dm_tx_message.DLC = 0x08;

	HAL_CAN_AddTxMessage(&hcan2, &dm_tx_message, dm_can_send_data, &send_mail_box);
}

/**
 * @brief 发送达妙电机使能
 */
void CAN_cmd_dm_j4310_mit_enable(void)
{
	uint32_t send_mail_box;

	dm_can_send_data[0] = 0xff;
	dm_can_send_data[1] = 0xff;
	dm_can_send_data[2] = 0xff;
	dm_can_send_data[3] = 0xff;
	dm_can_send_data[4] = 0xff;
	dm_can_send_data[5] = 0xff;
	dm_can_send_data[6] = 0xff;
	dm_can_send_data[7] = 0xfc;

	dm_tx_message.StdId = GIMBAL_ID;
	dm_tx_message.IDE = CAN_ID_STD;
	dm_tx_message.RTR = CAN_RTR_DATA;
	dm_tx_message.DLC = 0x08;

	HAL_CAN_AddTxMessage(&hcan2, &dm_tx_message, dm_can_send_data, &send_mail_box);
}

/**
 * @brief 发送达妙电机失能
 */
void CAN_cmd_dm_j4310_mit_disable(void)
{
	uint32_t send_mail_box;

	dm_can_send_data[0] = 0xff;
	dm_can_send_data[1] = 0xff;
	dm_can_send_data[2] = 0xff;
	dm_can_send_data[3] = 0xff;
	dm_can_send_data[4] = 0xff;
	dm_can_send_data[5] = 0xff;
	dm_can_send_data[6] = 0xff;
	dm_can_send_data[7] = 0xfd;

	dm_tx_message.StdId = GIMBAL_ID;
	dm_tx_message.IDE = CAN_ID_STD;
	dm_tx_message.RTR = CAN_RTR_DATA;
	dm_tx_message.DLC = 0x08;

	HAL_CAN_AddTxMessage(&hcan2, &dm_tx_message, dm_can_send_data, &send_mail_box);
}

/**
 * @brief 发送达妙电机保存位置零点
 */
void CAN_cmd_dm_j4310_mit_zero_cmd(void)
{
	uint32_t send_mail_box;

	dm_can_send_data[0] = 0xff;
	dm_can_send_data[1] = 0xff;
	dm_can_send_data[2] = 0xff;
	dm_can_send_data[3] = 0xff;
	dm_can_send_data[4] = 0xff;
	dm_can_send_data[5] = 0xff;
	dm_can_send_data[6] = 0xff;
	dm_can_send_data[7] = 0xFE;

	dm_tx_message.StdId = GIMBAL_ID;
	dm_tx_message.IDE = CAN_ID_STD;
	dm_tx_message.RTR = CAN_RTR_DATA;
	dm_tx_message.DLC = 0x08;

	HAL_CAN_AddTxMessage(&hcan2, &dm_tx_message, dm_can_send_data, &send_mail_box);
}

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_DJI_t *get_chassis_motor_measure_point(uint8_t i)
{
	return &motor_chassis[(i & 0x03)];
}

/**
  * @brief          返回云台电机 达妙电机数据指针
  * @param
  * @retval         电机数据指针
  */
const motor_measure_DM_t *get_DM_motor_measure_point(void)
{
	return &motor_DM;
}

/**
  * @brief          返回云台C板通信数据指针
  * @param
  * @retval         云台C板数据指针
  */
const boards_measure_t *get_boards_measure_point(void)
{
	return &boards_measure;
}