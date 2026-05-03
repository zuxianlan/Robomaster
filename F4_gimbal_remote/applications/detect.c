#include "detect.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

// 记录距离上一次收到数据过去了多少个周期
static uint16_t rc_cnt = 0;

void detect_rc_reset(void)
{
    // 收到数据，立刻清零
    rc_cnt = 0;
}

void detect_rc_tick(void)
{
    // 每次调用加1，防止数值溢出
    if(rc_cnt < 65535)
    {
        rc_cnt++;
    }
}

uint8_t is_rc_off(void)
{
    // 超过阈值则判定掉线
    return (rc_cnt >= RC_OFFLINE_LIMIT) ? 1 : 0;
}