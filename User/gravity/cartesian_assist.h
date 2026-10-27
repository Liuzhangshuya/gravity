#ifndef __CARTESIAN_ASSIST_H
#define __CARTESIAN_ASSIST_H

#include <stdint.h>

/*
 * 末端笛卡尔阻抗（各向异性）：
 *
 *   e = p_tcp - p_ref
 *   v = Jv * q_dot
 *   F = -K*e - D*v          K = diag(KX, KY, KZ)
 *   tau_model = Jv^T * F
 *   tau_motor = torque_sign * tau_model
 *
 * 水平 x/y 刚度大、竖直 z 刚度小，就是"水平大阻力、竖直小阻力"。
 * 只在 fsm_geforce_off（自由拖动）里叠加，且要调试器把
 * g_cart_assist_enable 写成 1 才生效；默认关闭时行为与原来完全一致。
 */

/* TCP 相对 {6} 的偏置 (m)。小臂末端就是 {6} 原点时保持 0；
 * 装了工具后按实际工具点修改，三个方向都要量。 */
#define CART_ASSIST_TCP_X 0.0f
#define CART_ASSIST_TCP_Y 0.0f
#define CART_ASSIST_TCP_Z 0.0f

/* 各向异性刚度 (N/m)。水平两个方向大，竖直方向小。 */
#define CART_ASSIST_KX 120.0f
#define CART_ASSIST_KY 120.0f
#define CART_ASSIST_KZ 30.0f

/* 各向异性阻尼 (N·s/m)，防止松手后来回摆。 */
#define CART_ASSIST_DX 15.0f
#define CART_ASSIST_DY 15.0f
#define CART_ASSIST_DZ 3.0f

/* 末端虚拟力上限 (N)。注意 J0 的力矩上限只有 0.6 N·m，
 * 水平方向 6~7 N 就会顶到限幅，所以不要一开始就调大。 */
#define CART_ASSIST_FMAX 6.0f

/* 打开时增益从 0 线性升到 1 的时间 (ms)。 */
#define CART_ASSIST_RAMP_MS 500U

/* 速度一阶低通系数 (0~1)，越小越平滑、滞后越大。 */
#define CART_ASSIST_VEL_ALPHA 0.35f

/* ---------------- 姿态（6D）阻抗，默认关闭 ----------------
 *
 *   e_R = vee(R * R_ref^T) / 2     轴角，底座系，rad
 *   omega = Jw * q_dot
 *   M = -Kr*e_R - Dr*omega         N·m
 *   tau = Jv^T*F + Jw^T*M
 *
 * 平移部分不受影响；只有 g_cart_assist_orient_enable=1 时才叠加。
 * 绕 x/y 相当于"保持工具水平"，绕 z 相当于"保持朝向"，z 先给软一点。
 */
#define CART_ASSIST_KRX 2.0f   /* N·m/rad */
#define CART_ASSIST_KRY 2.0f
#define CART_ASSIST_KRZ 1.0f

#define CART_ASSIST_DRX 0.30f  /* N·m·s/rad */
#define CART_ASSIST_DRY 0.30f
#define CART_ASSIST_DRZ 0.15f

/* 虚拟力矩上限 (N·m)。J0/J5 的力矩上限只有 0.6 N·m，
 * 力矩绕 z 几乎全压在 J0、绕 x 压在 J3/J5，所以先卡在 0.3。 */
#define CART_ASSIST_MMAX 0.30f

/* 调试变量：全部只读（除 enable/recapture），随控制周期刷新。
 * Keil Watch 里可以展开数组，也可以直接写 g_cart_assist_dbg.p[0]。 */
typedef struct
{
    float p[3];          /* 当前 TCP，底座 {0}，m */
    float p_ref[3];      /* 进入时捕获的参考点，m */
    float e[3];          /* p - p_ref，m */
    float v[3];          /* TCP 速度，m/s */
    float F[3];          /* 期望末端力，N */
    float tau_cart[6];   /* Jv^T F，电机侧，N·m */
    float tau_grav[6];   /* 叠加前的重力前馈，N·m */
    float tau_total[6];  /* 叠加后的 tor_set（限幅前），N·m */
    float J[3][6];       /* 位置雅可比，m/rad */
    float Jw[3][6];      /* 角速度雅可比，1/rad（各关节转轴，底座系） */
    float q[6];          /* 模型角，rad */
    float qd[6];         /* 模型角速度（滤波后），rad/s */
    float e_R[3];        /* 姿态误差，轴角，底座系，rad */
    float omega[3];      /* 末端角速度，底座系，rad/s */
    float M[3];          /* 姿态期望力矩，N·m */
    float tau_rot[6];    /* 姿态项贡献，模型侧，N·m */
    float gain;          /* 软启动增益 0~1 */
    uint32_t sat_cnt;        /* 总次数 = sat_force_cnt + sat_tau_cnt + sat_mom_cnt，Watch 写 0 清零 */
    uint32_t sat_force_cnt;  /* |F| 被 FMAX 裁剪的周期数（快速拖动时阻尼项也会顶到） */
    uint32_t sat_tau_cnt;    /* 关节阻抗力矩超过 0.8×上限的次数 */
    uint32_t sat_mom_cnt;    /* |M| 被 MMAX 裁剪的周期数 */
    uint8_t active;      /* 平移阻抗项是否正在输出 */
    uint8_t enabled;     /* 本周期是否允许（使能 && 浮动 && 未熔断） */
    uint8_t orient;      /* 本周期姿态项是否在输出 */
} CartAssistDebug_t;

extern CartAssistDebug_t g_cart_assist_dbg;

/* 调试器里写 1 打开、写 0 关闭。初值在 cartesian_assist.c 顶部；
 * 交付/正式验收时应为 0（默认关，行为与旧固件一致），调试时再写 1。 */
extern volatile uint8_t g_cart_assist_enable;

/* 调试器里写 1：下一周期把当前 TCP 和姿态重新记为 p_ref / R_ref
 *（增益不重新斜坡）。 */
extern volatile uint8_t g_cart_assist_recapture;

/* 全局增益缩放，1.0 = 头文件里的 K/D（K 和 D 同乘，阻尼比不变）。
 * 初值在 cartesian_assist.c 顶部；本文档里所有数值都按 1.0 给出。
 * 第一次试机建议 0.5，确认稳定后再回到 1.0，不用重新编译。 */
extern volatile float g_cart_assist_scale;

/* 刚度项缩放，默认 1.0。
 *   1.0        = K 和 D 都生效：像弹簧，松手回到 p_ref
 *   0.0        = 只剩阻尼：随动随停，停在放手的位置，但静止时不出力
 *   0~1        = 两者折中：有回弹，但回得慢
 * D 不受它影响，所以不存在 "K=0 时完全自由" 的突变。 */
extern volatile float g_cart_assist_k_scale;

/* 姿态项开关，默认 0。打开后平移部分参数不变，只多叠一层Jw^T M。 */
extern volatile uint8_t g_cart_assist_orient_enable;

/* 姿态项增益缩放，1.0 = 头文件里的 KR/DR。第一次试建议 0.3~0.5。 */
extern volatile float g_cart_assist_rot_scale;

/* 纯计算，不访问硬件：给定编码器角和电机侧角速度，输出电机侧阻抗力矩。
 * 供 PC 端数值验证使用。 */
void CartesianAssist_Step(const float encoder[6], const float encoder_vel[6],
                          uint8_t enabled, uint32_t now_ms,
                          float tau_out[6], CartAssistDebug_t *dbg);

/* 硬件接入：读取 motor[]，把阻抗力矩累加到 motor[i].cmd.tor_set。
 * float_mode = 0 时只更新调试量，不输出力矩。 */
void CartesianAssist_Run(uint8_t float_mode);

/* 清掉内部状态（参考点、速度滤波、软启动），下一次使能重新捕获。 */
void CartesianAssist_Reset(void);

#endif
