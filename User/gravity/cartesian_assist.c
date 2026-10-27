#include "cartesian_assist.h"
#include "gravity_config.h"
#include <math.h>

/* 主机测试只编译纯计算部分，不拉 HAL/电机头文件。 */
#ifndef CART_ASSIST_HOST_TEST
#include "main.h"
#include "dm4310_ctrl.h"
#include "motor_protect.h"
#endif

CartAssistDebug_t g_cart_assist_dbg;
volatile uint8_t g_cart_assist_enable = 1;
volatile uint8_t g_cart_assist_recapture = 0;
volatile float   g_cart_assist_scale = 1.0f;
volatile float   g_cart_assist_k_scale = 1.0f;
volatile uint8_t g_cart_assist_orient_enable = 1;
volatile float   g_cart_assist_rot_scale = 0.5f;

static const float kCartAssistK[3] = {
    CART_ASSIST_KX, CART_ASSIST_KY, CART_ASSIST_KZ
};
static const float kCartAssistD[3] = {
    CART_ASSIST_DX, CART_ASSIST_DY, CART_ASSIST_DZ
};
static const float kCartAssistKr[3] = {
    CART_ASSIST_KRX, CART_ASSIST_KRY, CART_ASSIST_KRZ
};
static const float kCartAssistDr[3] = {
    CART_ASSIST_DRX, CART_ASSIST_DRY, CART_ASSIST_DRZ
};
static const Vec3 kCartAssistTcp = {
    CART_ASSIST_TCP_X, CART_ASSIST_TCP_Y, CART_ASSIST_TCP_Z
};

static uint8_t  s_p_ref_valid;
static uint32_t s_start_ms;
static uint8_t  s_vel_filter_init;
static float    s_qd_filt[6];
static Vec3     s_p_ref;
static float    s_R_ref[3][3];

void CartesianAssist_Reset(void)
{
    s_p_ref_valid = 0;
    s_start_ms = 0;
    s_vel_filter_init = 0;
    s_p_ref.x = 0.0f;
    s_p_ref.y = 0.0f;
    s_p_ref.z = 0.0f;
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        s_qd_filt[i] = 0.0f;
    }
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            s_R_ref[r][c] = (r == c) ? 1.0f : 0.0f;
        }
    }
}

void CartesianAssist_Step(const float encoder[6], const float encoder_vel[6],
                          uint8_t enabled, uint32_t now_ms,
                          float tau_out[6], CartAssistDebug_t *dbg)
{
    float q[ROBOT_DOF];
    float qd_raw[ROBOT_DOF];
    float p[3];
    Mat4 frames[ROBOT_DOF];
    Vec3 p_tcp;
    float R[3][3];

    for (int i = 0; i < ROBOT_DOF; i++)
    {
        tau_out[i] = 0.0f;
    }

    /* 几何部分每周期都算，这样未使能时 Watch 里也能看到 p 和 J。 */
    GravityComp_EncoderToModelQ(encoder, q);
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        qd_raw[i] = GRAVITY_JOINT_MAP[i].position_sign * encoder_vel[i];
        dbg->q[i] = q[i];
        dbg->qd[i] = 0.0f;
    }

    GravityComp_ForwardKinematics(q, frames);
    p_tcp = GravityMath_TransformPoint(frames[5], kCartAssistTcp);
    p[0] = p_tcp.x;
    p[1] = p_tcp.y;
    p[2] = p_tcp.z;
    dbg->p[0] = p[0];
    dbg->p[1] = p[1];
    dbg->p[2] = p[2];

    for (int i = 0; i < ROBOT_DOF; i++)
    {
        Vec3 z_axis = GravityMath_GetZAxis(frames[i]);
        Vec3 origin = GravityMath_GetTranslation(frames[i]);
        Vec3 lever = GravityMath_Sub(p_tcp, origin);
        Vec3 column = GravityMath_Cross(z_axis, lever);
        dbg->J[0][i] = column.x;
        dbg->J[1][i] = column.y;
        dbg->J[2][i] = column.z;
        dbg->Jw[0][i] = z_axis.x;
        dbg->Jw[1][i] = z_axis.y;
        dbg->Jw[2][i] = z_axis.z;
    }

    /* {6} 相对底座的姿态，姿态阻抗用。 */
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            R[r][c] = frames[5].m[r][c];
        }
    }

    dbg->enabled = enabled;
    dbg->orient = 0;
    dbg->gain = 0.0f;
    dbg->active = 0;
    for (int j = 0; j < 3; j++)
    {
        dbg->e[j] = 0.0f;
        dbg->v[j] = 0.0f;
        dbg->F[j] = 0.0f;
        dbg->e_R[j] = 0.0f;
        dbg->omega[j] = 0.0f;
        dbg->M[j] = 0.0f;
    }
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        dbg->tau_cart[i] = 0.0f;
        dbg->tau_rot[i] = 0.0f;
    }

    if (!enabled)
    {
        /* 退出浮动/关闭开关：下一次使能重新抓参考点、重新软启动。 */
        s_p_ref_valid = 0;
        s_vel_filter_init = 0;
        s_start_ms = now_ms;
        return;
    }

    /* 末端速度反馈：电机侧角速度先按 position_sign 换成模型侧，再低通。 */
    if (!s_vel_filter_init)
    {
        for (int i = 0; i < ROBOT_DOF; i++)
        {
            s_qd_filt[i] = qd_raw[i];
        }
        s_vel_filter_init = 1;
    }
    else
    {
        for (int i = 0; i < ROBOT_DOF; i++)
        {
            s_qd_filt[i] += CART_ASSIST_VEL_ALPHA * (qd_raw[i] - s_qd_filt[i]);
        }
    }
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        dbg->qd[i] = s_qd_filt[i];
    }
    for (int j = 0; j < 3; j++)
    {
        float sum = 0.0f;
        for (int i = 0; i < ROBOT_DOF; i++)
        {
            sum += dbg->J[j][i] * s_qd_filt[i];
        }
        dbg->v[j] = sum;
    }
    for (int j = 0; j < 3; j++)
    {
        float sum = 0.0f;
        for (int i = 0; i < ROBOT_DOF; i++)
        {
            sum += dbg->Jw[j][i] * s_qd_filt[i];
        }
        dbg->omega[j] = sum;
    }

    if (!s_p_ref_valid)
    {
        s_p_ref.x = p_tcp.x;
        s_p_ref.y = p_tcp.y;
        s_p_ref.z = p_tcp.z;
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                s_R_ref[r][c] = R[r][c];
            }
        }
        s_p_ref_valid = 1;
        s_start_ms = now_ms;
        for (int i = 0; i < ROBOT_DOF; i++)
        {
            s_qd_filt[i] = qd_raw[i];
        }
    }
    if (g_cart_assist_recapture != 0)
    {
        g_cart_assist_recapture = 0;
        s_p_ref.x = p_tcp.x;
        s_p_ref.y = p_tcp.y;
        s_p_ref.z = p_tcp.z;
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                s_R_ref[r][c] = R[r][c];
            }
        }
    }

    uint32_t elapsed = now_ms - s_start_ms;
    float gain = (float)elapsed / (float)CART_ASSIST_RAMP_MS;
    if (gain > 1.0f)
    {
        gain = 1.0f;
    }

    float e[3];
    float F[3];
    e[0] = p[0] - s_p_ref.x;
    e[1] = p[1] - s_p_ref.y;
    e[2] = p[2] - s_p_ref.z;
    float scale = g_cart_assist_scale;
    if (scale < 0.0f)
    {
        scale = 0.0f;
    }
    /* k_scale 只缩刚度项：0 时只剩阻尼，松手停在哪就是哪（随动随停），
     * 静止时不出力；1 时是完整弹簧，松手回 p_ref。 */
    float ks = g_cart_assist_k_scale;
    if (ks < 0.0f)
    {
        ks = 0.0f;
    }
    for (int j = 0; j < 3; j++)
    {
        F[j] = -(scale * ks * kCartAssistK[j]) * e[j]
               - (scale * kCartAssistD[j]) * dbg->v[j];
    }

    float force_norm = sqrtf(F[0] * F[0] + F[1] * F[1] + F[2] * F[2]);
    if (force_norm > CART_ASSIST_FMAX)
    {
        float fscale = CART_ASSIST_FMAX / force_norm;
        F[0] *= fscale;
        F[1] *= fscale;
        F[2] *= fscale;
        dbg->sat_cnt++;
        dbg->sat_force_cnt++;
    }

    /* 姿态项：默认关闭，关闭时 M 保持 0，平移路径与之前完全一致。 */
    float M[3] = {0.0f, 0.0f, 0.0f};
    if (g_cart_assist_orient_enable != 0U)
    {
        float rs = g_cart_assist_rot_scale;
        if (rs < 0.0f)
        {
            rs = 0.0f;
        }

        /* C = R * R_ref^T；C 的反对称部分/2 就是底座系下的轴角误差。 */
        float C[3][3];
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                C[r][c] = R[r][0] * s_R_ref[c][0]
                        + R[r][1] * s_R_ref[c][1]
                        + R[r][2] * s_R_ref[c][2];
            }
        }
        float eR[3];
        eR[0] = 0.5f * (C[2][1] - C[1][2]);
        eR[1] = 0.5f * (C[0][2] - C[2][0]);
        eR[2] = 0.5f * (C[1][0] - C[0][1]);

        for (int j = 0; j < 3; j++)
        {
            M[j] = -(rs * kCartAssistKr[j]) * eR[j]
                   - (rs * kCartAssistDr[j]) * dbg->omega[j];
            dbg->e_R[j] = eR[j];
        }

        float mom_norm = sqrtf(M[0] * M[0] + M[1] * M[1] + M[2] * M[2]);
        if (mom_norm > CART_ASSIST_MMAX)
        {
            float mscale = CART_ASSIST_MMAX / mom_norm;
            M[0] *= mscale;
            M[1] *= mscale;
            M[2] *= mscale;
            dbg->sat_cnt++;
            dbg->sat_mom_cnt++;
        }
        dbg->orient = 1;
    }

    for (int i = 0; i < ROBOT_DOF; i++)
    {
        float tau_model = dbg->J[0][i] * F[0]
                        + dbg->J[1][i] * F[1]
                        + dbg->J[2][i] * F[2];
        float tau_rot = dbg->Jw[0][i] * M[0]
                      + dbg->Jw[1][i] * M[1]
                      + dbg->Jw[2][i] * M[2];
        tau_model = (tau_model + tau_rot) * gain;
        dbg->tau_rot[i] = tau_rot * gain;
        dbg->tau_cart[i] = tau_model;
        tau_out[i] = GRAVITY_JOINT_MAP[i].torque_sign * tau_model;
    }

    dbg->M[0] = M[0];
    dbg->M[1] = M[1];
    dbg->M[2] = M[2];

    dbg->p_ref[0] = s_p_ref.x;
    dbg->p_ref[1] = s_p_ref.y;
    dbg->p_ref[2] = s_p_ref.z;
    for (int j = 0; j < 3; j++)
    {
        dbg->e[j] = e[j];
        dbg->F[j] = F[j];
    }
    dbg->gain = gain;
    dbg->active = 1;
}

#ifndef CART_ASSIST_HOST_TEST

void CartesianAssist_Run(uint8_t float_mode)
{
    float encoder[ROBOT_DOF];
    float encoder_vel[ROBOT_DOF];
    float tau[ROBOT_DOF];
    uint8_t enabled;

    /* 三重门：调试器打开 + 当前在浮动状态 + 系统没有熔断。 */
    enabled = (uint8_t)((g_cart_assist_enable != 0U) &&
                        (float_mode != 0U) &&
                        (global_system_halt_flag == 0U));

    for (int i = 0; i < ROBOT_DOF; i++)
    {
        encoder[i] = motor[i].para.pos;
        encoder_vel[i] = motor[i].para.vel;
        g_cart_assist_dbg.tau_grav[i] = motor[i].cmd.tor_set;
    }

    CartesianAssist_Step(encoder, encoder_vel, enabled, HAL_GetTick(), tau,
                         &g_cart_assist_dbg);

    for (int i = 0; i < ROBOT_DOF; i++)
    {
        motor[i].cmd.tor_set += tau[i];
        g_cart_assist_dbg.tau_total[i] = motor[i].cmd.tor_set;

        /* 关节力矩接近上限时记录一次，提示应该降低 K 或换姿态。 */
        if (enabled != 0U)
        {
            float limit = motor_limits[i].max_tor;
            if (tau[i] > 0.8f * limit || tau[i] < -0.8f * limit)
            {
                g_cart_assist_dbg.sat_cnt++;
                g_cart_assist_dbg.sat_tau_cnt++;
            }
        }
    }
}

#endif
