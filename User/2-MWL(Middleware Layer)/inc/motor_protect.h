#ifndef __MOTOR_PROTECT_H
#define __MOTOR_PROTECT_H

#include "main.h"

typedef struct {
    float max_tor;      // 最大力矩
    float max_vel;      // 最大速度
    float max_pos;      // 最大位置
    float min_pos;      // 最小位置
} motor_limit_t;

extern const motor_limit_t motor_limits[6];
extern volatile uint8_t global_system_halt_flag;

/* 熔断来源诊断（只读）：
 * global_halt_reason: 1=反馈不可用 2=硬超速 3=硬越界 4=重力力矩非有限
 *                     5=状态机反馈检查
 * global_halt_fsm_state: 触发熔断时状态机所处的状态（0xFF 表示未记录）
 */
extern volatile uint8_t  global_halt_reason;
extern volatile uint8_t  global_halt_axis;
extern volatile float    global_halt_value;
extern volatile uint8_t  global_halt_fsm_state;
extern volatile uint8_t  global_halt_state[6];
extern volatile uint32_t global_halt_fb_age[6];

void motor_protect_run(void);
uint8_t motor_speed_limited(void);
void motor_disable_detect(void);


#endif

