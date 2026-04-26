#ifndef CHASSIS_REMOTE_2_DETECT_H
#define CHASSIS_REMOTE_2_DETECT_H

#include "stdint.h"

// 遥控器掉线阈值
#define RC_OFFLINE_LIMIT  60

// 清零计时
void detect_rc_reset(void);

// 累加计时
void detect_rc_tick(void);

// 查询遥控器是否掉线。返回 1：掉线，返回 0：正常
uint8_t is_rc_off(void);

#endif //CHASSIS_REMOTE_2_DETECT_H