#ifndef CHASSIS_REMOTE_2_CHASSIS_BEHAVIOUR_H
#define CHASSIS_REMOTE_2_CHASSIS_BEHAVIOUR_H
#include "struct_typedef.h"
#include "chassis_behaviour.h"
#include "chassis_task.h"
typedef enum
{
    // CHASSIS_ZERO_FORCE,                   //底盘无力, 跟没上电那样
    // CHASSIS_NO_MOVE,                      //底盘保持不动
    CHASSIS_ROTATION,                     //底盘小陀螺
    CHASSIS_FOLLOW_YAW,   //正常步兵底盘跟随云台

    CHASSIS_NO_FOLLOW_YAW,                //底盘不跟随角度，角度是开环的，但轮子是有速度环
} chassis_behaviour_e;

extern chassis_behaviour_e chassis_behaviour_mode;
/**
 * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
 * @param[in]      chassis_move_mode: 底盘数据
 * @retval         none
 */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
 * @brief 根据行为模式调用对应的控制函数
 */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

#endif //CHASSIS_REMOTE_2_CHASSIS_BEHAVIOUR_H