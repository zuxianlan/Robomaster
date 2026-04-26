#include "chassis_task.h"
#include "Vofa.h"
#include "main.h"
#include "cmsis_os.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include <math.h>
#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "chassis_behaviour.h"
#include "remote_control.h"

/**
 * @brief 底盘小陀螺(ROTATION)行为控制函数
 * @note vx和vy正常读取遥控器，wz强制设为3.0f
 */
static void chassis_rotation_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief 普通不跟随模式控制函数
 */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief 普通跟随云台控制函数
 */
static void chassis_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

chassis_behaviour_e chassis_behaviour_mode;
/**
 * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
 * @param[in]      chassis_move_mode: 底盘数据
 * @retval         none
 */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    //遥控器设置模式
    if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNE1]))
    {
        chassis_behaviour_mode = CHASSIS_ROTATION;      //小陀螺
    }
    else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNE1]))
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;     //底盘不跟随云台
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNE1]))
    {
        chassis_behaviour_mode = CHASSIS_FOLLOW_YAW;        //底盘跟随云台
    }
}

/**
 * @brief 根据行为模式调用对应的控制函数
 */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;

    if (chassis_behaviour_mode == CHASSIS_ROTATION)
    {
        chassis_rotation_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_FOLLOW_YAW)
    {
        chassis_follow_yaw_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
    }
    else
    {
        *vx_set = 0.0f;
        *vy_set = 0.0f;
        *wz_set = 0.0f;
    }

}
/**
 * @brief 底盘小陀螺(ROTATION)行为控制函数
 * @note vx和vy正常读取遥控器，wz强制设为2.0f
 */
static void chassis_rotation_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;

    int16_t ch2 = chassis_move_rc_to_vector->chassis_RC->rc.ch[2]; // 左右
    int16_t ch3 = chassis_move_rc_to_vector->chassis_RC->rc.ch[3]; // 前后

    fp32 vx_set_temp = (fp32)ch2 * CHASSIS_VX_RC_SEN;
    fp32 vy_set_temp = (fp32)ch3 * CHASSIS_VX_RC_SEN;

    // 死区判断
    if (vx_set_temp < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_temp > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN) {
        vx_set_temp = 0.0f;
    }
    if (vy_set_temp < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vy_set_temp > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN) {
        vy_set_temp = 0.0f;
    }

    *vx_set = vx_set_temp / 4.0f;
    *vy_set = vy_set_temp / 4.0f;

    //以 ??? 的速度旋转
    *wz_set = 2.0f;
    //*wz_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[0] / 660.0f * 2.0f;
}

/**
 * @brief 底盘不跟随云台控制函数
 */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;

    int16_t ch2 = chassis_move_rc_to_vector->chassis_RC->rc.ch[2];
    int16_t ch3 = chassis_move_rc_to_vector->chassis_RC->rc.ch[3];
    //int16_t ch0 = chassis_move_rc_to_vector->chassis_RC->rc.ch[0];
    int16_t ch1 = chassis_move_rc_to_vector->chassis_RC->rc.ch[1];

    fp32 vx_set_temp = (fp32)ch2 * CHASSIS_VX_RC_SEN;
    fp32 vy_set_temp = (fp32)ch3 * CHASSIS_VX_RC_SEN;

    if (vx_set_temp < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_temp > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN) vx_set_temp = 0.0f;
    if (vy_set_temp < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vy_set_temp > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN) vy_set_temp = 0.0f;

    *vx_set = vx_set_temp / 4.0f;
    *vy_set = vy_set_temp / 4.0f;
    *wz_set = 0.0f;
    //*wz_set = (float)ch1 / 660.0f * 2.0f;
}

/**
 * @brief 底盘跟随云台控制函数
 */
static void chassis_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL) return;

    int16_t ch2 = chassis_move_rc_to_vector->chassis_RC->rc.ch[2];
    int16_t ch3 = chassis_move_rc_to_vector->chassis_RC->rc.ch[3];

    fp32 vx_set_temp = (fp32)ch2 * CHASSIS_VX_RC_SEN;
    fp32 vy_set_temp = (fp32)ch3 * CHASSIS_VX_RC_SEN;

    if (vx_set_temp < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_temp > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN) vx_set_temp = 0.0f;
    if (vy_set_temp < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vy_set_temp > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN) vy_set_temp = 0.0f;

    *vx_set = vx_set_temp / 4.0f;
    *vy_set = vy_set_temp / 4.0f;

    *wz_set = gimbal_control.yaw_follow_omega_set / 10.0f;
    if (fabs(*wz_set) < 0.2)    {*wz_set = 0.0f;}

    // float data[1];
    // data[0] = *wz_set;
    // VOFA_Transmit_JustFloat(data, 1);
}