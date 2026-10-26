#include "dm4310_ctrl.h"
#include "dm4310_drv.h"

#include "fsm.h"
#include "math.h"
#include "motor_function.h"
#include "gravity.h"
#include "motor_protect.h"

uint8_t lock_flag;
static uint8_t motor_pos_init_reset_request = 1;
static uint8_t motor_pos_run_first_run = 1;

/* motor_pos_init 复位目标位置(单位: rad)，按关节0~5顺序配置 */
static const float motor_reset_target_pos_init[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
/* motor_pos_run 复位目标位置(单位: rad)，按关节0~5顺序配置 */
static const float motor_reset_target_pos_run[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

void motor_pos_run_reset(void)
{
    motor_pos_run_first_run = 1;
}

void motor_pos_init_reset(void)
{
    motor_pos_init_reset_request = 1;
}

void motor_wait_feedback_update(void)
{
    for (int i = 0; i < motor_num; i++) {
        motor[i].cmd.pos_set = motor[i].para.pos;
        motor[i].cmd.vel_set = 0.0f;
        motor[i].cmd.tor_set = 0.0f;
        motor[i].cmd.kp_set = 0.0f;
        motor[i].cmd.kd_set = 0.0f;
    }
    ctrl_set();
    ctrl_send();
}

void motor_pos_reset(void) 
{
    uint8_t mode_id = 0;// 0: MIT模式   1: 位置速度模式   2: 速度模式
    for (int i = 0; i < 3; i++) {
        save_pos_zero(&hcan1, motor[i].id, mode_id);
    }
    for (int i = 3; i < 6; i++) {
        save_pos_zero(&hcan2, motor[i].id, mode_id);
    }
}


void motor_lock(void) //柔性锁定
{ 
    static float motor_lock_pos[6];
    static float motor_lock_kp[6];
    float gravity_torque[6];
    if (!motor_feedback_ready()) {
        global_system_halt_flag = 1;
        return;
    }
    Gravity_Arm_Calculate(gravity_torque);
    
    if (lock_flag == 0) {
        for(int i = 0; i < motor_num; i++) {
            motor_lock_pos[i] = motor[i].para.pos;
            motor_lock_kp[i] = MOTOR_LOCK_KP_MIN;
        }
        lock_flag = 1;
    }
    
    for(int i = 0; i < motor_num; i++) {
        float target_kp = MOTOR_LOCK_KP_MIN
                        + MOTOR_LOCK_KP_PER_TORQUE * fabsf(gravity_torque[i]);
        if (target_kp > MOTOR_LOCK_KP_MAX) target_kp = MOTOR_LOCK_KP_MAX;
        motor_lock_kp[i] += MOTOR_LOCK_KP_SMOOTHING * (target_kp - motor_lock_kp[i]);
        Set_MIT_PVT(&motor[i], motor_lock_pos[i], 0.0f, gravity_torque[i]);
        Set_MIT_PD(&motor[i], motor_lock_kp[i], MOTOR_LOCK_KD);
    }
		//motor_par_send();
}


static float home_position[motor_num];
static float home_vel_filt[motor_num];
static uint32_t home_start_ms;
static uint32_t home_last_ms;
static uint32_t home_stable_ms;
static uint8_t home_stable;

/* 回零失败诊断：只做记录，不参与任何控制判断。
 * 熔断后可在调试器 Watch 窗口直接查看，确认是哪一轴、哪一条判定没有满足。
 *
 * home_fail_reason:
 *   1 = 10 s 总超时   2 = 跟随误差 > 0.5 rad   3 = 起始目标/当前位置越界
 *   4 = 反馈不可用    5 = 两次调用间隔 > 100 ms
 *
 * home_fail_mask: 每轴占 3 位，bit0=位置超差(>0.05), bit1=速度超差(滤波值>0.1),
 *                 bit2=内部轨迹未到位(>0.001)。轴 i 的位移为 3*i。
 * home_fail_vel 记录的是原始速度，bit1 用的是完成判定实际使用的滤波值。
 */
volatile uint8_t  home_fail_reason;
volatile uint32_t home_fail_mask;
volatile float    home_fail_pos_err[motor_num];
volatile float    home_fail_vel[motor_num];
volatile float    home_fail_traj_err[motor_num];
volatile uint8_t  home_fail_state[motor_num];

/* 200 ms 稳定窗口被打破的累计次数（内部轨迹到位后才开始统计），
 * 用来区分“一直没到位”和“反复被某一轴打断”。 */
volatile uint32_t home_reset_pos_cnt;
volatile uint32_t home_reset_vel_cnt;
volatile uint32_t home_reset_traj_cnt;
volatile uint8_t  home_reset_axis;
static uint8_t    home_settle_counting;

static void home_record_failure(uint8_t reason, const float target[motor_num])
{
    uint32_t mask = 0;
    for (int i = 0; i < motor_num; i++)
    {
        float pos_error = fabsf(target[i] - motor[i].para.pos);
        float vel = fabsf(motor[i].para.vel);
        float traj_error = fabsf(target[i] - home_position[i]);

        home_fail_pos_err[i] = pos_error;
        home_fail_vel[i] = vel;
        home_fail_traj_err[i] = traj_error;
        home_fail_state[i] = (uint8_t)motor[i].para.state;

        if (pos_error > 0.05f)  mask |= (1UL << (i * 3));
        if (fabsf(home_vel_filt[i]) > 0.1f) mask |= (1UL << (i * 3 + 1));
        if (traj_error > 0.001f) mask |= (1UL << (i * 3 + 2));
    }
    home_fail_mask = mask;
    home_fail_reason = reason;
}

// 开始回零：检查目标范围，并记录当前实际位置作为起点。
static uint8_t motor_home_start(const float target[motor_num], uint32_t now)
{
    home_fail_reason = 0;
    home_fail_mask = 0;
    home_reset_pos_cnt = 0;
    home_reset_vel_cnt = 0;
    home_reset_traj_cnt = 0;
    home_reset_axis = 0xFF;
    home_settle_counting = 0;

    for (int i = 0; i < motor_num; i++)
    {
        home_vel_filt[i] = 0.0f;
    }

    for (int i = 0; i < motor_num; i++)
    {
        if (target[i] < motor_limits[i].min_pos ||
            target[i] > motor_limits[i].max_pos)
        {
            home_record_failure(3, target);
            global_system_halt_flag = 1;
            return 0;
        }

        if (motor[i].para.pos < motor_limits[i].min_pos ||
            motor[i].para.pos > motor_limits[i].max_pos)
        {
            home_record_failure(3, target);
            global_system_halt_flag = 1;
            return 0;
        }

        home_position[i] = motor[i].para.pos;
    }

    home_start_ms = now;
    home_last_ms = now;
    home_stable = 0;
    return 1;
}

// 全部关节实际到位、速度足够小，并持续稳定 200 ms 才算完成。
static uint8_t motor_home_arrived(const float target[motor_num], uint32_t now)
{
    // 速度判定用一阶低通值(约 40 ms 时间常数)，避免单周期反馈毛刺清零 200 ms 窗口。
    for (int i = 0; i < motor_num; i++)
    {
        home_vel_filt[i] += 0.2f * (motor[i].para.vel - home_vel_filt[i]);
    }

    // 内部轨迹全部到位后才开始统计稳定窗口被打断的原因。
    if (home_settle_counting == 0)
    {
        uint8_t all_traj_done = 1;
        for (int i = 0; i < motor_num; i++)
        {
            if (fabsf(target[i] - home_position[i]) > 0.001f)
            {
                all_traj_done = 0;
                break;
            }
        }
        if (all_traj_done)
        {
            home_settle_counting = 1;
            home_reset_pos_cnt = 0;
            home_reset_vel_cnt = 0;
            home_reset_traj_cnt = 0;
        }
    }

    for (int i = 0; i < motor_num; i++)
    {
        float position_error = target[i] - motor[i].para.pos;
        float trajectory_error = target[i] - home_position[i];

        if (fabsf(position_error) > 0.05f)
        {
            if (home_settle_counting) { home_reset_pos_cnt++; home_reset_axis = (uint8_t)i; }
            home_stable = 0;
            return 0;
        }
        if (fabsf(home_vel_filt[i]) > 0.1f)
        {
            if (home_settle_counting) { home_reset_vel_cnt++; home_reset_axis = (uint8_t)i; }
            home_stable = 0;
            return 0;
        }
        if (fabsf(trajectory_error) > 0.001f)
        {
            if (home_settle_counting) { home_reset_traj_cnt++; home_reset_axis = (uint8_t)i; }
            home_stable = 0;
            return 0;
        }
    }

    if (home_stable == 0)
    {
        home_stable = 1;
        home_stable_ms = now;
    }

    if (now - home_stable_ms >= 200U)
    {
        return 1;
    }
    return 0;
}

// 每周期调用：检查状态，向目标移动一步，然后检查是否到位。
static uint8_t motor_home_run(const float target[motor_num], uint8_t restart,
                              float speed, float kp, float kd)
{
    uint32_t now = HAL_GetTick();

    if (global_system_halt_flag != 0 || motor_feedback_ready() == 0)
    {
        home_record_failure(4, target);
        global_system_halt_flag = 1;
        return 0;
    }

    if (restart != 0)
    {
        if (motor_home_start(target, now) == 0)
        {
            return 0;
        }
    }

    // 回零总时间不能超过 10 s，两次调用间隔不能超过 100 ms。
    uint32_t elapsed = now - home_start_ms;
    uint32_t step_ms = now - home_last_ms;
    home_last_ms = now;
    if (elapsed > 10000U || step_ms > 100U)
    {
        home_record_failure((elapsed > 10000U) ? 1 : 5, target);
        global_system_halt_flag = 1;
        return 0;
    }

    // 前 500 ms 逐渐增加速度和增益，避免开始回零时突然拉动。
    float ramp = (float)elapsed / 500.0f;
    if (ramp > 1.0f)
    {
        ramp = 1.0f;
    }
    float step = speed * ((float)step_ms * 0.001f) * ramp;

    for (int i = 0; i < motor_num; i++)
    {
        // 实际位置跟不上内部目标时停止，避免误差继续积累。
        if (fabsf(home_position[i] - motor[i].para.pos) > 0.5f)
        {
            home_record_failure(2, target);
            global_system_halt_flag = 1;
            return 0;
        }

        float error = target[i] - home_position[i];
        if (motor_speed_limited() == 0)
        {
            if (error > step)
            {
                home_position[i] += step;
            }
            else if (error < -step)
            {
                home_position[i] -= step;
            }
            else
            {
                home_position[i] = target[i];
            }
        }

        motor[i].cmd.pos_set = home_position[i];
        motor[i].cmd.vel_set = 0.0f;
        motor[i].cmd.kp_set = kp * ramp;
        motor[i].cmd.kd_set = kd * ramp;
    }

    return motor_home_arrived(target, now);
}

uint8_t motor_pos_init(void)
{
    // kp 需按残余重力力矩选：kp=40 时 J2 的稳态误差约 0.035 rad，保留容差余量。
    // kd 按 kd ∝ sqrt(kp) 同步提高，保持与原 kp=15/kd=1 相近的阻尼比。
    uint8_t finished = motor_home_run(motor_reset_target_pos_init,
                                      motor_pos_init_reset_request, 0.6f, 40.0f, 1.6f);
    if (global_system_halt_flag == 0)
    {
        motor_pos_init_reset_request = 0;
    }
    return finished;
}

uint8_t motor_pos_run(void)
{
    // 与 init 使用相同的刚度：目标同为 0，残余重力力矩相同。
    uint8_t finished = motor_home_run(motor_reset_target_pos_run,
                                      motor_pos_run_first_run, 0.5f, 40.0f, 1.6f);
    if (global_system_halt_flag == 0)
    {
        motor_pos_run_first_run = 0;
    }
    return finished;
}

/* 标定快照：只读取反馈和最终指令，不改变任何控制量。
 * 由 fsm.c 在 motor_protect_run() 之后调用，所以记录的是实际下发的值。 */
volatile uint8_t  calib_capture;
volatile uint8_t  calib_count;
volatile float    calib_pos[CALIB_POSE_MAX][motor_num];
volatile float    calib_vel[CALIB_POSE_MAX][motor_num];
volatile float    calib_tor[CALIB_POSE_MAX][motor_num];
volatile float    calib_tor_set[CALIB_POSE_MAX][motor_num];
volatile float    calib_pos_set[CALIB_POSE_MAX][motor_num];
volatile float    calib_kp[CALIB_POSE_MAX][motor_num];
volatile uint32_t calib_ms[CALIB_POSE_MAX];

void calib_capture_run(void)
{
    if (calib_capture == 0)
    {
        return;
    }

    // 先清标志，避免抓取过程中被调试器重复触发。
    calib_capture = 0;

    // 熔断后下发的全是安全模式值，没有标定意义，直接丢弃这次触发。
    if (global_system_halt_flag != 0)
    {
        return;
    }

    if (calib_count >= CALIB_POSE_MAX)
    {
        return;
    }

    uint8_t n = calib_count;
    for (int i = 0; i < motor_num; i++)
    {
        calib_pos[n][i]     = motor[i].para.pos;
        calib_vel[n][i]     = motor[i].para.vel;
        calib_tor[n][i]     = motor[i].para.tor;
        calib_tor_set[n][i] = motor[i].cmd.tor_set;
        calib_pos_set[n][i] = motor[i].cmd.pos_set;
        calib_kp[n][i]      = motor[i].cmd.kp_set;
    }
    calib_ms[n] = HAL_GetTick();
    calib_count = (uint8_t)(n + 1);
}
