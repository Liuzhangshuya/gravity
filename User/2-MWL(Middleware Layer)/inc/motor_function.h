#ifndef __MOTOR_FUNCTION_H
#define __MOTOR_FUNCTION_H

/*
 * 锁定模式的动态刚度参数：
 *
 * 目标 P 增益计算公式：
 * target_kp = MOTOR_LOCK_KP_MIN
 *           + MOTOR_LOCK_KP_PER_TORQUE * fabsf(当前重力补偿力矩);
 * 随后受 MOTOR_LOCK_KP_MAX 限制，并经平滑系数逐周期更新。
 */

/* 最小 P 增益（基础锁定刚度）。
 * 当前重力补偿只有 20% 且模型未校准，锁定必须靠 P 承担主要载荷：
 * 残余重力约 1.4 N·m 时，kp=40 对应约 0.035 rad 的稳态偏差（与回零一致）。
 * 等重力模型校准、补偿增益提高后，可以再逐步降低该值以获得更好的柔顺性。 */
#define MOTOR_LOCK_KP_MIN         40.0f

/* 力矩到 P 增益的比例系数。补偿力矩绝对值每增加 1 N·m，目标 P 增加 0.35。 */
#define MOTOR_LOCK_KP_PER_TORQUE  0.35f

/* P 增益上限，防止重力补偿力矩较大时锁定刚度过高。 */
#define MOTOR_LOCK_KP_MAX         50.0f

/* P 增益平滑系数，范围通常为 0~1。0.30 表示每周期走完目标差值的 30%。 */
#define MOTOR_LOCK_KP_SMOOTHING   0.30f

/* 锁定模式的 D 增益（阻尼）。按 kd ∝ sqrt(kp) 与回零的 kp=40/kd=1.6 一致，
 * 原来的 0.02 几乎无阻尼，下坠时会摆动并触发超速保护。 */
#define MOTOR_LOCK_KD             1.6f

/* 自由拖动（fsm_geforce_off）的阻尼：kp=0，只保留重力前馈 + 阻尼。
 * 手推时几乎只有阻尼力，松手后阻尼让手臂停下；重力补偿增益接近 1 时，
 * 残余重力很小，手臂可以停在放开的位置，不必再切到 lock。 */
#define MOTOR_FLOAT_KD            1.0f

extern uint8_t lock_flag;

/* 回零失败诊断（只读），含义见 motor_function.c。 */
extern volatile uint8_t  home_fail_reason;
extern volatile uint32_t home_fail_mask;
extern volatile float    home_fail_pos_err[];
extern volatile float    home_fail_vel[];
extern volatile float    home_fail_traj_err[];
extern volatile uint8_t  home_fail_state[];
extern volatile uint32_t home_reset_pos_cnt;
extern volatile uint32_t home_reset_vel_cnt;
extern volatile uint32_t home_reset_traj_cnt;
extern volatile uint8_t  home_reset_axis;

/* 标定快照（只读诊断，不参与控制）。
 * 用法：
 *   1. Watch 窗口添加 calib_count 和 calib_capture；
 *   2. 手臂摆好、停稳后，把 calib_capture 写成 1；
 *   3. 下一控制周期抓取一帧，calib_capture 自动清 0，calib_count 加 1；
 *   4. 展开 calib_pos / calib_vel / calib_tor / calib_tor_set / calib_pos_set / calib_kp，
 *      第 calib_count-1 行就是刚抓的姿态；
 *   5. 重新开始：把 calib_count 写 0。
 * 抓取点在 motor_protect_run() 之后，记录的是最终实际下发的指令。 */
#define CALIB_POSE_MAX 8
extern volatile uint8_t  calib_capture;
extern volatile uint8_t  calib_count;
extern volatile float    calib_pos[CALIB_POSE_MAX][motor_num];
extern volatile float    calib_vel[CALIB_POSE_MAX][motor_num];
extern volatile float    calib_tor[CALIB_POSE_MAX][motor_num];
extern volatile float    calib_tor_set[CALIB_POSE_MAX][motor_num];
extern volatile float    calib_pos_set[CALIB_POSE_MAX][motor_num];
extern volatile float    calib_kp[CALIB_POSE_MAX][motor_num];
extern volatile uint32_t calib_ms[CALIB_POSE_MAX];
void calib_capture_run(void);

void motor_pos_reset(void);
void motor_lock(void);
uint8_t motor_pos_init(void);
uint8_t motor_pos_run(void) ;
void motor_pos_init_reset(void);
void motor_pos_run_reset(void);
void motor_wait_feedback_update(void);

#endif
