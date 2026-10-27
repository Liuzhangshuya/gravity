"""Host-side check for User/gravity/cartesian_assist.c.

Compiles the pure-computation part with gcc (CART_ASSIST_HOST_TEST) and checks:

  1. FK at the physical home pose against the value computed offline;
  2. the analytic Jacobian columns against central differences of FK;
  3. p_ref capture, 500 ms soft start, e/F signs and tau = J^T F * torque_sign;
  4. disabling clears the output and the next enable recaptures p_ref.

No Python is needed to use the firmware; this script is only a build-time check.
"""
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GRAVITY = ROOT / 'User' / 'gravity'

TEST_C = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "gravity_config.h"
#include "cartesian_assist.h"

static Vec3 fk_tcp(const float q[6])
{
    Mat4 frames[6];
    Vec3 tcp = {0.0f, 0.0f, 0.0f};
    GravityComp_ForwardKinematics(q, frames);
    return GravityMath_TransformPoint(frames[5], tcp);
}

static void q_to_encoder(const float q[6], float enc[6])
{
    for (int i = 0; i < 6; i++)
    {
        enc[i] = GRAVITY_JOINT_MAP[i].encoder_zero
               + GRAVITY_JOINT_MAP[i].position_sign * q[i];
    }
}

/* Analytic J vs central difference of FK, at any pose. */
static void check_jacobian(const float q[6], const CartAssistDebug_t *dbg)
{
    const float h = 1e-3f;
    for (int i = 0; i < 6; i++)
    {
        float qp[6], qm[6];
        memcpy(qp, q, sizeof(qp));
        memcpy(qm, q, sizeof(qm));
        qp[i] += h;
        qm[i] -= h;
        Vec3 a = fk_tcp(qp);
        Vec3 b = fk_tcp(qm);
        float fd[3] = {(a.x - b.x) / (2.0f * h),
                       (a.y - b.y) / (2.0f * h),
                       (a.z - b.z) / (2.0f * h)};
        for (int j = 0; j < 3; j++)
        {
            float err = fabsf(fd[j] - dbg->J[j][i]);
            if (err > 2e-4f)
            {
                printf("J mismatch i=%d row=%d fd=%f an=%f err=%f\n",
                       i, j, fd[j], dbg->J[j][i], err);
                assert(0);
            }
        }
    }
}

int main(void)
{
    float qh[6], enc[6], vel[6] = {0.0f};
    float tau[6];
    CartAssistDebug_t dbg;

    /* 不依赖固件里为调试改过的默认值，测试固定用 scale=1.0。 */
    g_cart_assist_enable = 0;
    g_cart_assist_scale = 1.0f;
    g_cart_assist_k_scale = 1.0f;
    g_cart_assist_orient_enable = 0;
    g_cart_assist_rot_scale = 1.0f;
    memset(&dbg, 0, sizeof(dbg));

    for (int i = 0; i < 6; i++)
    {
        qh[i] = -GRAVITY_JOINT_MAP[i].position_sign
              * GRAVITY_JOINT_MAP[i].encoder_zero;
    }
    q_to_encoder(qh, enc);

    CartesianAssist_Reset();

    /* 1. geometry while disabled: home TCP from the offline calculation. */
    CartesianAssist_Step(enc, vel, 0, 0, tau, &dbg);
    assert(dbg.active == 0);
    assert(fabsf(dbg.p[0] - 0.0821f) < 5e-4f);
    assert(fabsf(dbg.p[1] + 0.0099f) < 5e-4f);
    assert(fabsf(dbg.p[2] - 0.1018f) < 5e-4f);
    check_jacobian(qh, &dbg);
    for (int i = 0; i < 6; i++)
    {
        assert(fabsf(tau[i]) < 1e-6f);
    }

    /* 2. first enabled cycle captures p_ref, gain starts at 0 -> no torque. */
    CartesianAssist_Step(enc, vel, 1, 0, tau, &dbg);
    assert(dbg.active == 1);
    assert(dbg.gain < 1e-6f);
    assert(fabsf(dbg.p_ref[0] - dbg.p[0]) < 1e-6f);
    for (int i = 0; i < 6; i++)
    {
        assert(fabsf(tau[i]) < 1e-6f);
    }

    /* 3. after the 500 ms ramp the spring is fully on and e is still zero. */
    CartesianAssist_Step(enc, vel, 1, 600, tau, &dbg);
    assert(fabsf(dbg.gain - 1.0f) < 1e-6f);
    for (int j = 0; j < 3; j++)
    {
        assert(fabsf(dbg.e[j]) < 2e-6f);
        assert(fabsf(dbg.F[j]) < 1e-5f);
    }

    /* 4. move one joint: F must oppose e, tau must equal J^T F * torque_sign. */
    {
        float q2[6], enc2[6];
        memcpy(q2, qh, sizeof(q2));
        q2[1] += 0.05f;
        q_to_encoder(q2, enc2);

        CartesianAssist_Step(enc2, vel, 1, 700, tau, &dbg);

        Vec3 p2 = fk_tcp(q2);
        float e_exp[3] = {p2.x - dbg.p_ref[0], p2.y - dbg.p_ref[1],
                          p2.z - dbg.p_ref[2]};
        for (int j = 0; j < 3; j++)
        {
            assert(fabsf(dbg.e[j] - e_exp[j]) < 2e-6f);
        }
        for (int j = 0; j < 3; j++)
        {
            float f_exp = -((j == 0) ? CART_ASSIST_KX :
                            (j == 1) ? CART_ASSIST_KY : CART_ASSIST_KZ) * e_exp[j];
            assert(fabsf(dbg.F[j] - f_exp) < 2e-5f);
        }
        for (int i = 0; i < 6; i++)
        {
            float t_model = dbg.J[0][i] * dbg.F[0]
                          + dbg.J[1][i] * dbg.F[1]
                          + dbg.J[2][i] * dbg.F[2];
            float t_exp = GRAVITY_JOINT_MAP[i].torque_sign * t_model;
            assert(fabsf(tau[i] - t_exp) < 2e-5f);
        }
        check_jacobian(q2, &dbg);

        /* 4b. runtime gain scale: 0.5 halves F, negative clamps to 0. */
        g_cart_assist_scale = 0.5f;
        CartesianAssist_Step(enc2, vel, 1, 700, tau, &dbg);
        for (int j = 0; j < 3; j++)
        {
            float f_exp = -0.5f * ((j == 0) ? CART_ASSIST_KX :
                                   (j == 1) ? CART_ASSIST_KY : CART_ASSIST_KZ)
                          * e_exp[j];
            assert(fabsf(dbg.F[j] - f_exp) < 2e-5f);
        }
        g_cart_assist_scale = -1.0f;
        CartesianAssist_Step(enc2, vel, 1, 700, tau, &dbg);
        for (int j = 0; j < 3; j++)
        {
            assert(fabsf(dbg.F[j]) < 1e-6f);
        }
        for (int i = 0; i < 6; i++)
        {
            assert(fabsf(tau[i]) < 1e-6f);
        }
        g_cart_assist_scale = 1.0f;
    }

    /* 4c. force clamp: a large offset must be cut to FMAX on the norm. */
    {
        float q3[6], enc3[6];
        uint32_t force_before = dbg.sat_force_cnt;
        memcpy(q3, qh, sizeof(q3));
        q3[0] += 1.0f;   /* 底座回转 1 rad，末端位移足够大（约 76 mm） */
        q_to_encoder(q3, enc3);

        CartesianAssist_Step(enc3, vel, 1, 750, tau, &dbg);

        float fn = sqrtf(dbg.F[0] * dbg.F[0] + dbg.F[1] * dbg.F[1]
                       + dbg.F[2] * dbg.F[2]);
        assert(fabsf(fn - CART_ASSIST_FMAX) < 1e-4f);
        assert(dbg.sat_force_cnt > force_before);
        /* 裁剪只缩不放，方向必须和 -K*e 一致。 */
        for (int j = 0; j < 3; j++)
        {
            float f_raw = -((j == 0) ? CART_ASSIST_KX :
                            (j == 1) ? CART_ASSIST_KY : CART_ASSIST_KZ) * dbg.e[j];
            assert(f_raw * dbg.F[j] >= 0.0f);
            assert(fabsf(dbg.F[j]) <= fabsf(f_raw) + 1e-6f);
        }
    }

    /* 5. disabling wipes the output; enabling again recaptures p_ref. */
    CartesianAssist_Step(enc, vel, 0, 800, tau, &dbg);
    assert(dbg.active == 0);
    assert(dbg.gain == 0.0f);
    for (int i = 0; i < 6; i++)
    {
        assert(tau[i] == 0.0f);
    }

    CartesianAssist_Step(enc, vel, 1, 900, tau, &dbg);
    assert(dbg.active == 1);
    assert(dbg.gain < 1e-6f);
    assert(fabsf(dbg.p_ref[0] - dbg.p[0]) < 1e-6f);

    /* 6. orientation term: Jw = joint axes, e_R from a wrist rotation,
     *    tau = Jv^T F + Jw^T M, moment clamp. */
    {
        Mat4 fr[6];
        float q5[6], enc5[6];

        GravityComp_ForwardKinematics(qh, fr);

        CartesianAssist_Step(enc, vel, 0, 1000, tau, &dbg);   /* clear state */
        g_cart_assist_orient_enable = 1;
        g_cart_assist_rot_scale = 1.0f;
        CartesianAssist_Step(enc, vel, 1, 1000, tau, &dbg);   /* capture */
        CartesianAssist_Step(enc, vel, 1, 1600, tau, &dbg);   /* gain = 1 */

        for (int i = 0; i < 6; i++)
        {
            assert(fabsf(dbg.Jw[0][i] - fr[i].m[0][2]) < 1e-5f);
            assert(fabsf(dbg.Jw[1][i] - fr[i].m[1][2]) < 1e-5f);
            assert(fabsf(dbg.Jw[2][i] - fr[i].m[2][2]) < 1e-5f);
        }

        /* rotate J5 by 0.05 rad: TCP stays put, orientation changes about z5. */
        memcpy(q5, qh, sizeof(q5));
        q5[5] += 0.05f;
        q_to_encoder(q5, enc5);
        CartesianAssist_Step(enc5, vel, 1, 1700, tau, &dbg);

        float z5[3] = {fr[5].m[0][2], fr[5].m[1][2], fr[5].m[2][2]};
        assert(dbg.orient == 1);
        for (int j = 0; j < 3; j++)
        {
            assert(fabsf(dbg.e_R[j] - 0.05f * z5[j]) < 1e-4f);
            float k = (j == 0) ? CART_ASSIST_KRX :
                      (j == 1) ? CART_ASSIST_KRY : CART_ASSIST_KRZ;
            assert(fabsf(dbg.M[j] + k * dbg.e_R[j]) < 1e-4f);
        }
        for (int i = 0; i < 6; i++)
        {
            float t_model = dbg.J[0][i] * dbg.F[0] + dbg.J[1][i] * dbg.F[1]
                          + dbg.J[2][i] * dbg.F[2]
                          + dbg.Jw[0][i] * dbg.M[0] + dbg.Jw[1][i] * dbg.M[1]
                          + dbg.Jw[2][i] * dbg.M[2];
            assert(fabsf(tau[i]
                         - GRAVITY_JOINT_MAP[i].torque_sign * t_model) < 2e-5f);
            assert(fabsf(dbg.tau_rot[i] - dbg.Jw[0][i] * dbg.M[0]
                         - dbg.Jw[1][i] * dbg.M[1]
                         - dbg.Jw[2][i] * dbg.M[2]) < 2e-5f);
        }

        /* 1 rad about J5 must clamp |M| to MMAX. */
        uint32_t mom_before = dbg.sat_mom_cnt;
        q5[5] = qh[5] + 1.0f;
        q_to_encoder(q5, enc5);
        CartesianAssist_Step(enc5, vel, 1, 1800, tau, &dbg);
        float mn = sqrtf(dbg.M[0] * dbg.M[0] + dbg.M[1] * dbg.M[1]
                       + dbg.M[2] * dbg.M[2]);
        assert(fabsf(mn - CART_ASSIST_MMAX) < 1e-4f);
        assert(dbg.sat_mom_cnt > mom_before);

        g_cart_assist_orient_enable = 0;
    }

    /* 7. k_scale = 0: spring removed, damping kept -> park-anywhere mode. */
    {
        float q6[6], enc6[6], enc_v[6], vel0[6] = {0.0f};
        memcpy(q6, qh, sizeof(q6));
        q6[1] += 0.05f;                 /* ~25 mm 位移 */
        q_to_encoder(q6, enc6);
        g_cart_assist_k_scale = 0.0f;

        /* 7a. displaced but at rest -> zero force: no spring, so it stays. */
        CartesianAssist_Step(enc, vel0, 0, 2000, tau, &dbg);
        CartesianAssist_Step(enc, vel0, 1, 2000, tau, &dbg);   /* capture home */
        CartesianAssist_Step(enc6, vel0, 1, 2600, tau, &dbg);  /* moved, still */
        assert(fabsf(dbg.gain - 1.0f) < 1e-6f);
        assert(sqrtf(dbg.e[0] * dbg.e[0] + dbg.e[1] * dbg.e[1]
                     + dbg.e[2] * dbg.e[2]) > 1e-3f);
        for (int j = 0; j < 3; j++)
        {
            assert(fabsf(dbg.F[j]) < 1e-6f);
        }

        /* 7b. moving at constant speed -> F = -D*v, no K*e term. */
        for (int i = 0; i < 6; i++)
        {
            enc_v[i] = 0.02f;
        }
        CartesianAssist_Step(enc6, enc_v, 0, 3000, tau, &dbg);
        CartesianAssist_Step(enc6, enc_v, 1, 3000, tau, &dbg); /* filter = speed */
        CartesianAssist_Step(enc6, enc_v, 1, 3600, tau, &dbg);
        for (int j = 0; j < 3; j++)
        {
            float v_exp = 0.0f;
            for (int i = 0; i < 6; i++)
            {
                v_exp += dbg.J[j][i] * GRAVITY_JOINT_MAP[i].position_sign * 0.02f;
            }
            float d = (j == 0) ? CART_ASSIST_DX :
                      (j == 1) ? CART_ASSIST_DY : CART_ASSIST_DZ;
            assert(fabsf(dbg.v[j] - v_exp) < 1e-5f);
            assert(fabsf(dbg.F[j] + d * v_exp) < 2e-5f);
        }
        g_cart_assist_k_scale = 1.0f;
    }

    puts("PASS: FK, Jacobian vs FD, p_ref, soft start, F sign, tau = J^T F, "
         "FMAX clamp, 6D: Jw/M/e_R, k_scale=0 damping-only");
    return 0;
}
'''


WRAPPER_TEST_C = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "cartesian_assist.h"
#include "dm4310_ctrl.h"
#include "motor_protect.h"

motor_t motor[6];
const motor_limit_t motor_limits[6] = {
    {0.6f}, {2.9f}, {2.3f}, {1.2f}, {1.2f}, {0.6f}
};
volatile uint8_t global_system_halt_flag = 0;

static uint32_t now_ms = 0;
uint32_t HAL_GetTick(void) { return now_ms; }

int main(void)
{
    float before[6];
    for (int i = 0; i < 6; i++)
    {
        motor[i].para.pos = 0.0f;
        motor[i].para.vel = 0.0f;
        motor[i].cmd.tor_set = 0.1f;
        before[i] = motor[i].cmd.tor_set;
    }

    /* Default off: the wrapper must leave tor_set untouched. */
    CartesianAssist_Reset();
    g_cart_assist_enable = 0;
    g_cart_assist_scale = 1.0f;
    CartesianAssist_Run(1);
    for (int i = 0; i < 6; i++)
    {
        assert(fabsf(motor[i].cmd.tor_set - before[i]) < 1e-6f);
    }

    /* Enabled but not in float: still untouched. */
    g_cart_assist_enable = 1;
    CartesianAssist_Run(0);
    for (int i = 0; i < 6; i++)
    {
        assert(fabsf(motor[i].cmd.tor_set - before[i]) < 1e-6f);
    }

    /* Enabled in float: gravity torque is recorded and the ramp starts at 0,
     * so the first cycles add nothing yet. */
    CartesianAssist_Run(1);
    for (int i = 0; i < 6; i++)
    {
        assert(fabsf(g_cart_assist_dbg.tau_grav[i] - before[i]) < 1e-6f);
        assert(fabsf(motor[i].cmd.tor_set - before[i]) < 1e-6f);
    }
    assert(g_cart_assist_dbg.active == 1);

    /* Halt blocks the impedance term. */
    g_cart_assist_enable = 0;
    global_system_halt_flag = 1;
    CartesianAssist_Run(1);
    assert(g_cart_assist_dbg.enabled == 0);

    puts("PASS: wrapper gating, gravity snapshot, no output while disabled");
    return 0;
}
'''

STUB_MAIN_H = r'''
#ifndef MAIN_H
#define MAIN_H
#include <stdint.h>
uint32_t HAL_GetTick(void);
#endif
'''

STUB_MOTOR_CTRL_H = r'''
#ifndef DM4310_CTRL_H
#define DM4310_CTRL_H
#include "main.h"
typedef struct
{
    struct { float pos; float vel; } para;
    struct { float tor_set; } cmd;
} motor_t;
#define motor_num 6
extern motor_t motor[motor_num];
#endif
'''

STUB_MOTOR_PROTECT_H = r'''
#ifndef MOTOR_PROTECT_H
#define MOTOR_PROTECT_H
#include "main.h"
typedef struct { float max_tor; } motor_limit_t;
extern const motor_limit_t motor_limits[motor_num];
extern volatile uint8_t global_system_halt_flag;
#endif
'''


def main():
    with tempfile.TemporaryDirectory(prefix='cart_assist_test_') as tmp:
        folder = Path(tmp)
        (folder / 'test.c').write_text(TEST_C, encoding='utf-8')
        exe = folder / 'test.exe'
        cmd = [
            'gcc', '-std=c11', '-O0', '-Wall', '-Wextra',
            '-DCART_ASSIST_HOST_TEST',
            '-I', str(GRAVITY),
            str(folder / 'test.c'),
            str(GRAVITY / 'gravity_math.c'),
            str(GRAVITY / 'gravity_comp.c'),
            str(GRAVITY / 'cartesian_assist.c'),
            '-lm', '-o', str(exe),
        ]
        subprocess.run(cmd, check=True)
        subprocess.run([str(exe)], check=True)

        # Second check: the hardware wrapper, with stub HAL/motor headers.
        stubs = folder / 'stubs'
        stubs.mkdir()
        (stubs / 'main.h').write_text(STUB_MAIN_H, encoding='utf-8')
        (stubs / 'dm4310_ctrl.h').write_text(STUB_MOTOR_CTRL_H, encoding='utf-8')
        (stubs / 'motor_protect.h').write_text(STUB_MOTOR_PROTECT_H, encoding='utf-8')
        (folder / 'wrapper_test.c').write_text(WRAPPER_TEST_C, encoding='utf-8')
        exe2 = folder / 'wrapper_test.exe'
        cmd2 = [
            'gcc', '-std=c11', '-O0', '-Wall', '-Wextra',
            '-I', str(stubs), '-I', str(GRAVITY),
            str(folder / 'wrapper_test.c'),
            str(GRAVITY / 'gravity_math.c'),
            str(GRAVITY / 'gravity_comp.c'),
            str(GRAVITY / 'cartesian_assist.c'),
            '-lm', '-o', str(exe2),
        ]
        subprocess.run(cmd2, check=True)
        subprocess.run([str(exe2)], check=True)


if __name__ == '__main__':
    sys.exit(main())
