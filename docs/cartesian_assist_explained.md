# 末端各向异性阻抗：代码、原理与设计原因

配套文档：实测记录 [work_log_2026-10-27.md](work_log_2026-10-27.md)、
操作手册 [cartesian_assist_debug.md](cartesian_assist_debug.md)、
雅可比原理 [jacobian_principle.md](jacobian_principle.md)。

本文回答三个问题：**每一段代码在算什么、为什么这样算、当初为什么选这个方案**。
所有代码片段都来自 [cartesian_assist.c](../User/gravity/cartesian_assist.c)、
[cartesian_assist.h](../User/gravity/cartesian_assist.h) 和
[fsm.c](../User/1-APL%28Application%20Layer%29/src/fsm.c)，数值按 home 姿态给出。

---

## 1. 总体结构：它挂在哪一层

```
编码器 pos/vel
   │  position_sign, encoder_zero
   ▼
模型角 q, q̇ ──► 正运动学 ──► frames[0..5] ──► p, R
   │                              │
   │                              ├──► Jv = z × r,  Jw = z
   ▼                              ▼
速度低通 ──► v, ω            e = p − p_ref,  e_R = 0.5·vee(R·R_refᵀ)
   │                              │
   └──────────► F = −K·e − D·v ◄─┘        M = −Kr·e_R − Dr·ω
                     │                            │
                     ├──► |F| ≤ FMAX              ├──► |M| ≤ MMAX
                     ▼                            ▼
              τ_model = Jvᵀ·F  +  Jwᵀ·M
                     │  × gain（软启动）, × torque_sign
                     ▼
              motor[i].tor_set += τ        （叠加在重力前馈之后）
```

固件里的调用顺序（[fsm.c:191-198](../User/1-APL%28Application%20Layer%29/src/fsm.c#L191-L198)）：

```c
Gravity_Arm_Run();
CartesianAssist_Run((uint8_t)(fsm->state == fsm_geforce_off));
float ramp = fminf((float)(HAL_GetTick() - compensation_start_ms) / 500.0f, 1.0f);
for (int i = 0; i < motor_num; i++) {
    motor[i].cmd.tor_set *= ramp;
}
```

**为什么必须是这个顺序**：`Gravity_Arm_Run()` 先算出抵消重力的前馈，
`CartesianAssist_Run()` 再把阻抗力矩加在它上面。阻抗项是**叠加**，不是替代，
所以重力补偿模型、参数、`ROBOT_PARAMETERS_READY` 都不需要动。

**为什么只能挂在 `fsm_geforce_off`**：该状态在 [fsm.c:157-162](../User/1-APL%28Application%20Layer%29/src/fsm.c#L157-L162)
把 `kp_set = 0`、`kd_set = MOTOR_FLOAT_KD`，电机内部只剩阻尼，没有位置环。
只有在这种"可被手推动"的状态下，叠加一个力矩才有意义；如果 kp 不为 0，
位置环会把末端拉回 `pos_set`，和阻抗力矩对着干，两边都失真。

**为什么没有用逆运动学**：我们只需要把"末端想要多大力"翻译成关节力矩，
这是一个**前向**映射 `τ = JᵀF`，不需要求 `J⁻¹`，也不会在奇异位形处除零。
要做"给一个末端位置、算出关节角"才需要逆解（学长那套速度级控制的路线），
那是另一种架构，不在本次范围。

---

## 2. 从编码器到末端位姿

### 2.1 编码器 → 模型角

```c
GravityComp_EncoderToModelQ(encoder, q);
```

每个关节在 `GRAVITY_JOINT_MAP[i]` 里配置了 `encoder_zero` 和 `position_sign`：

```
q_i = position_sign_i · (encoder_i − encoder_zero_i)
```

**原理**：编码器读数是一个带零点和方向的原始量；模型（DH 参数、重力模型、
雅可比）用的是**模型角**。home 是编码器全 0，所以 `q_home = −position_sign·encoder_zero`，
在 home 姿态下 `q = (−0.300, −1.500, −1.650, 0.200, −0.010, 0)` rad。

**原因**：方向（`position_sign`）和零点（`encoder_zero`）在装配时由实物决定，
不能写死在公式里；分开配置，换装配/换零位只改表，不改算法。

### 2.2 正运动学与 TCP

```c
GravityComp_ForwardKinematics(q, frames);
p_tcp = GravityMath_TransformPoint(frames[5], kCartAssistTcp);
```

`frames[i]` 是第 i 个关节坐标系相对底座 `{0}` 的位姿；`frames[5]` 是末端 `{6}`。
若装了工具，`kCartAssistTcp` 是工具点相对 `{6}` 的偏置 `(x, y, z)`，
`TransformPoint` 把它从 `{6}` 变到 `{0}`。

**原因**：阻抗要作用在**手真正抓的那个点**上。TCP 偏置错了，`p_ref`、`J`
和手感的对应关系全错——力的作用点不对，杠杆臂也就不对。
没有工具时保持 0。

---

## 3. 雅可比：`Jv` 和 `Jw`

### 3.1 代码

```c
for (int i = 0; i < ROBOT_DOF; i++)
{
    Vec3 z_axis = GravityMath_GetZAxis(frames[i]);
    Vec3 origin = GravityMath_GetTranslation(frames[i]);
    Vec3 lever  = GravityMath_Sub(p_tcp, origin);
    Vec3 column = GravityMath_Cross(z_axis, lever);
    dbg->J[0][i] = column.x;
    dbg->J[1][i] = column.y;
    dbg->J[2][i] = column.z;
    dbg->Jw[0][i] = z_axis.x;
    dbg->Jw[1][i] = z_axis.y;
    dbg->Jw[2][i] = z_axis.z;
}
```

### 3.2 每一列是什么

关节 i 以 `q̇_i` 转动时，末端点绕轴 `z_i` 做圆周运动，半径向量
`r_i = p_tcp − o_i`，所以线速度

```
v_i = ω × r = (z_i · q̇_i) × r_i
```

因此线速度雅可比第 i 列

```
Jv[:, i] = z_i × (p_tcp − o_i)        [m/rad]
```

角速度部分更简单：每个转动关节直接贡献 `z_i · q̇_i`，所以

```
Jw[:, i] = z_i                        [1/rad]
```

**单位与含义**：`Jv` 的一列是"其余关节不动、第 i 个关节以 1 rad/s 正转时，
末端线速度（m/s）的贡献"。`|z × r|` 等于末端到该关节轴线的**垂直距离**，
所以末端的运动方向永远垂直于"转轴—末端"这个平面，符合右手定则。

### 3.3 为什么取 `frames[i]` 而不是 `frames[i-1]`

本仓库的 `A_i` 定义为

```
A_i = Rx(α_i) · Tx(a_i) · Rz(θ_i) · Tz(d_i)
```

关节 i 的 `Rz(θ_i)` 写在 `A_i` 里，所以它的轴与轴上点要取 **`frames[i]`**：
J0 用 `frames[0]`，J5 用 `frames[5]`。Craig 改进 DH 与标准 DH 的索引差一级，
照抄教材很容易整体错一列。判断方法只有一个：**和正运动学的中心差分对**——
本仓库的主机测试就是这么做的（第 12 节），允许误差 2×10⁻⁴ m/rad。

### 3.4 `z × r` 的几何直觉

- r 与 z 平行（末端正好在轴线上）→ 叉乘为 0，该关节瞬间推不动末端。
  home 下 J5 的列就是零，因为 TCP 正好在腕部回转轴上。
- 末端离轴越远，同样的转角产生越大的线位移。
- 换姿态时 `z_i` 和 `r_i` 都变，所以同一个关节在不同姿态下的贡献不同，
  但**刚度 K 是任务空间的属性，不随姿态变**——这是第 4 节的关键。

---

## 4. 阻抗律：`F = −K·e − D·v`

### 4.1 代码

```c
float ks = g_cart_assist_k_scale;
if (ks < 0.0f) { ks = 0.0f; }
for (int j = 0; j < 3; j++)
{
    F[j] = -(scale * ks * kCartAssistK[j]) * e[j]
           - (scale * kCartAssistD[j]) * dbg->v[j];
}
```

其中 `scale` 是调试用的总增益（`g_cart_assist_scale`），`ks` 是刚度缩放
（第 10 节）。默认 `scale = ks = 1`。

### 4.2 各项的单位与作用

| 项 | 表达式 | 单位 | 作用 |
| --- | --- | --- | --- |
| 弹簧 | `−K·e` | N/m × m = N | 偏离参考点就回推，静态刚度的来源 |
| 阻尼 | `−D·v` | N·s/m × m/s = N | 抑制来回摆，快速拖动时的"黏滞感" |
| 各向异性 | `K = diag(120, 120, 30)` | — | 水平/竖直阻力比 4:1 |
| 阻尼各向异性 | `D = diag(15, 15, 3)` | — | 水平/竖直 5:1 |

### 4.3 静态平衡：`e = K⁻¹·F_hand`

静止时 `v = 0`，所以 `F = −K·e`。手推着不动时，关节空间里
"阻抗力矩 + 手的作用力矩"必须平衡：

```
JᵀF + JᵀF_hand = 0   ⟹   F = −F_hand   ⟹   e = K⁻¹·F_hand
```

（`Jᵀ` 是 6×3，一般在工作空间内列满秩，所以可以约掉。）

这个式子解释了几件在实机上看起来"奇怪"的事：

1. **静态力等于手力，不是控制器给的 6 N**。`K` 决定"这个力要推多远"，
   不决定力的上限。想推 6 N，手就必须出 6 N；`scale=2` 只是把 50 mm 变成 25 mm。
2. **等位移看力**：`F_hand = K·e`，水平 25 mm → 3 N，竖直 25 mm → 0.75 N，比值 4。
3. **等力看位移**：同样的手力，竖直位移是水平的 4 倍。
4. **静态竖直偏移一定有竖直力在维持**。式子里没有关节角、没有零位姿态；
   折叠零位只决定关节怎么分配力矩，不决定末端停在哪。实测 60 mm 水平推时
   `e_z = −45 mm`、`F_z = +1.19 N`，就是手上带了 1.19 N 向下的分力，
   被 30 N/m 的软轴放大成 40 mm 左右。几何本身撑不住一个静态偏移。

### 4.4 为什么定义在底座系，而不是工具系

`K` 是底座 `{0}` 下的对角阵：水平永远指世界水平，竖直永远指重力方向。
如果定义在工具系，手腕一转，"水平大阻力"就会跟着工具一起转，
和作业要求的"水平/竖直"不是一回事。这也是为什么 `e`、`v`、`F`
全部在 `{0}` 下计算——维数统一，叠加和限幅才有意义。

### 4.5 为什么没有积分项

阻抗的目标是"手出多少力，末端让多少"，稳态误差就是**特性**，不是缺陷。
加积分会把手松开后的残余位移也消掉，等于把弹簧变成"位置伺服"，
既破坏了各向异性的直观手感，也容易在关节摩擦/间隙下产生极限环。
要让手臂停在某处，用 `k_scale=0`（第 10 节），不是加积分。

---

## 5. 从末端力到关节力矩：`τ = JᵀF`

### 5.1 代码

```c
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
```

### 5.2 为什么是转置

虚功原理：末端位移 `δp = J·δq`，力做的功必须等于关节力矩做的功：

```
Fᵀ·δp = τᵀ·δq   ⟹   Fᵀ·J·δq = τᵀ·δq   ⟹   τ = Jᵀ·F
```

所以**不需要求逆**：`Jᵀ` 永远有定义，奇异位形处也只是某些方向的力
映射不到关节（或映射很大），不会出现除零。这正符合"力矩级阻抗"的定位。

### 5.3 `torque_sign` 是什么

`tau_model` 是模型侧（DH 正方向）的力矩；电机收到的 `tor_set` 是驱动器侧的量。
两者可能因为电机安装方向/减速器方向差一个符号，所以每个关节单独配置
`torque_sign`，在最后一步乘上去：

```
tau_motor = torque_sign · tau_model
```

`position_sign` 管"编码器 → 模型角"，`torque_sign` 管"模型力矩 → 电机指令"，
两者都在 `GRAVITY_JOINT_MAP` 里，和重力补偿共用同一套约定。

### 5.4 力矩怎么分到各关节

home 姿态下（实测表见调试手册第 5 节）：

| 方向 | 1 N 对应的主要关节力矩 | 6 N 时的占用 |
| --- | --- | --- |
| y | J0：0.0821 N·m/N | 0.49 N·m，占 0.6 上限的 **82%** |
| z | J2：0.2012 N·m/N | 1.21 N·m，占 2.3 上限的 52% |
| z | J4：0.0776 N·m/N | 0.47 N·m，占 1.2 上限的 39% |

**这就是 `FMAX = 6 N` 的由来**：再大，home 姿态下 J0 会先顶到 0.6 N·m，
保护会介入，力矩分布也会失真。`FMAX` 不是随便定的手感参数，是**安全性约束**。

### 5.5 为什么限幅在 `F` 层，而且是范数

```c
float force_norm = sqrtf(F[0]*F[0] + F[1]*F[1] + F[2]*F[2]);
if (force_norm > CART_ASSIST_FMAX)
{
    float fscale = CART_ASSIST_FMAX / force_norm;
    F[0] *= fscale;
    F[1] *= fscale;
    F[2] *= fscale;
    dbg->sat_cnt++;
    dbg->sat_force_cnt++;
}
```

- 在 `F` 层限幅，力矩层就自动受限，且限幅的含义与"手感的力"一致；
- 用**范数**而不是逐轴裁剪：逐轴裁剪会改变力的方向，`F` 可能不再与 `e` 反号，
  静态关系 `e = K⁻¹F` 就被破坏；范数裁剪只缩大小、保方向。
- 代价是：阻尼项和弹簧项**一起**被缩。快速拖动或回程时，`D·v` 很大，
  一旦 `|F|` 顶到 6 N，阻尼也被削，所以在边界附近会略微欠阻尼——这正是
  实测"回 `p_ref` 时有一点点抖"的原因（见第 11 节）。

### 5.6 为什么还要单独统计关节力矩

```c
if (tau[i] > 0.8f * limit || tau[i] < -0.8f * limit)
{
    g_cart_assist_dbg.sat_cnt++;
    g_cart_assist_dbg.sat_tau_cnt++;
}
```

`|F| ≤ 6 N` 只是"末端力"的保证；不同姿态下同一个 `F` 分到 J0 的比例不同，
可能某个关节先到 0.8×上限。`sat_tau_cnt` 是这个早期预警：
它涨说明该姿态/该方向不能再加力，应降低 `scale` 或换姿态，而不是继续加大 K。

---

## 6. 速度估计

### 6.1 代码

```c
qd_raw[i] = GRAVITY_JOINT_MAP[i].position_sign * encoder_vel[i];
...
if (!s_vel_filter_init)
{
    for (int i = 0; i < ROBOT_DOF; i++) { s_qd_filt[i] = qd_raw[i]; }
    s_vel_filter_init = 1;
}
else
{
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        s_qd_filt[i] += CART_ASSIST_VEL_ALPHA * (qd_raw[i] - s_qd_filt[i]);
    }
}
```

### 6.2 为什么要滤波

- 驱动器上报的速度来自编码器差分，含量化噪声；`F = −D·v` 会把噪声
  直接放大成力矩抖动（`D=15` 时 0.01 m/s 的噪声就是 0.15 N 的抖动）。
- 低通把高频压掉，代价是滞后。离散一阶滤波 `y += α(x−y)` 在 100 Hz、
  `α = 0.35` 时，时间常数约 `−dt/ln(1−α) ≈ 23 ms`，截止约 7 Hz。
- **原因**：23 ms 对慢速人机拖动够用，又不会让阻尼在快速换向时相位落后太多。
  如果抖动还大，先降 `scale`（K 和 D 同乘），不要单独把 α 调得很小——
  那会把阻尼变成"延迟的力"，更容易振荡。

### 6.3 为什么不用位置差分自己算

`motor[i].para.vel` 已经是驱动器算好的速度，再差分一次只会多一层噪声和延迟；
这里只做符号变换和低通。方向必须乘 `position_sign`，因为 `v = J·q̇` 用的是
模型侧角速度。

---

## 7. 参考点与软启动

### 7.1 参考点

```c
if (!s_p_ref_valid)
{
    s_p_ref = p_tcp;
    s_R_ref = R;
    s_p_ref_valid = 1;
    s_start_ms = now_ms;
    ...
}
```

- 第一次允许输出时捕获当前 TCP 和姿态作为 `p_ref / R_ref`；
- `g_cart_assist_recapture = 1` 在下一周期重新捕获（不重新斜坡）；
- 退出浮动/关闭开关时 `s_p_ref_valid = 0`，下次进浮动重新捕获。

**原因**：`p_ref` 必须是"阻抗打开的那一瞬"的位置，而不是一个固定 home。
这样在任意姿态进入浮动，手臂都不会被猛地拽回原点。重捕获用于"拖到一个新位置，
把它当作新的平衡点"。

### 7.2 软启动

```c
uint32_t elapsed = now_ms - s_start_ms;
float gain = (float)elapsed / (float)CART_ASSIST_RAMP_MS;
if (gain > 1.0f) { gain = 1.0f; }
...
tau_model = (tau_model + tau_rot) * gain;
```

**原因**：使能瞬间如果直接给满力矩，是一阶跃输入，手臂会被"拽一下"，
也可能激发抖动。500 ms 线性升到 1，把阶跃变成斜坡。
注意 `gain` 只乘阻抗项，不乘重力前馈——重力补偿原本就有自己的启动斜坡
（[fsm.c:195](../User/1-APL%28Application%20Layer%29/src/fsm.c#L195)，只在固件启动后 500 ms 内有效）。

---

## 8. 门控与安全默认值

```c
enabled = (uint8_t)((g_cart_assist_enable != 0U) &&
                    (float_mode != 0U) &&
                    (global_system_halt_flag == 0U));
```

三重门：

| 条件 | 作用 |
| --- | --- |
| `g_cart_assist_enable` | 人显式打开；设计默认 0（调试期临时置 1） |
| `float_mode`（`fsm_geforce_off`） | 只在 kp=0 的浮动状态叠加 |
| `!global_system_halt_flag` | 系统熔断时立即不输出 |

**为什么默认关**：交付的固件在没人打开开关时，行为与旧固件**逐行等价**；
也避免复位后手臂在无人扶持时突然多出一股力。验收/演示时在 Watch 里写 1 即可，
不需要重编译。

**为什么关闭时也要算几何**：`p`、`J`、`Jw` 每周期都算，Watch 里可以看到，
便于在不使能的情况下先核对坐标和雅可比，再打开开关。

---

## 9. 姿态（6D）阻抗

### 9.1 代码

```c
/* C = R * R_ref^T */
for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
        C[r][c] = R[r][0] * s_R_ref[c][0]
                + R[r][1] * s_R_ref[c][1]
                + R[r][2] * s_R_ref[c][2];

float eR[3];
eR[0] = 0.5f * (C[2][1] - C[1][2]);
eR[1] = 0.5f * (C[0][2] - C[2][0]);
eR[2] = 0.5f * (C[1][0] - C[0][1]);

for (int j = 0; j < 3; j++)
{
    M[j] = -(rs * kCartAssistKr[j]) * eR[j]
           - (rs * kCartAssistDr[j]) * dbg->omega[j];
}
```

### 9.2 `e_R` 的推导

设当前姿态 `R = ΔR · R_ref`，则 `C = R·R_refᵀ = ΔR`。
绕单位轴 `n` 转 `θ` 时

```
ΔR = I + sinθ·[n]× + (1−cosθ)·[n]×²
```

代入 `0.5·vee(C − Cᵀ)`：

```
e_R = sinθ · n
```

小角度时 `e_R ≈ θ·n`，即"轴角"向量，方向是转轴、模长是转角（rad）。

**为什么用轴角而不是欧拉角**：欧拉角有万向锁、且三个角耦合，
小误差下也不线性；轴角从旋转矩阵直接取，无奇异，方向明确。
**为什么在底座系**：`ΔR` 是左乘（在底座系里转），而 `Jw` 的列也是底座系下的
关节轴，两者一致。若改成 `R_refᵀ·R`（工具系），就必须把 `Jw` 也转到工具系，
否则 `M` 和 `ω` 不在同一坐标系，力矩方向会错。

**已知局限**：`e_R = sinθ·n`，`θ > 90°` 后幅值反而变小，只适合小角度柔顺，
不适合大角度回零。这是刻意保持的线性化，换取小误差下的稳定手感。

### 9.3 `Jwᵀ·M` 与限幅

```c
float mom_norm = sqrtf(M[0]*M[0] + M[1]*M[1] + M[2]*M[2]);
if (mom_norm > CART_ASSIST_MMAX)
{
    float mscale = CART_ASSIST_MMAX / mom_norm;
    ...
    dbg->sat_mom_cnt++;
}
```

与 `F` 同样的理由：范数限幅保方向。`MMAX = 0.30 N·m` 是因为
绕底座 z 的力矩几乎全部由 J0 承担（`Jw[2][0] = 1`），而 J0 上限只有 0.6 N·m；
取 0.3 留 50% 余量。绕 x/y 的力矩主要落在 J3–J5，J5 上限同样只有 0.6 N·m。

`Jwᵀ·M` 的虚功推导与 `Jvᵀ·F` 完全一样：`Mᵀ·δθ = τᵀ·δq`，`δθ = Jw·δq`。

---

## 10. 回弹还是随动随停：`k_scale`

### 10.1 冲突从哪来

- 弹簧模式（`k_scale=1`）：`F = −K·e − D·v`。松手后 `e ≠ 0`，力把手臂拉回 `p_ref`。
- 原固件的浮动：`kp=0` + 重力前馈，拖到哪停在哪。

这两件事在"固定参考点"下**不能同时成立**：随动随停要求静止时力为零，
而弹簧在静止时恰好出力最大（与 `e` 成正比）。

### 10.2 `k_scale` 为什么只缩 K

```c
F[j] = -(scale * ks * kCartAssistK[j]) * e[j]
       - (scale * kCartAssistD[j]) * dbg->v[j];
```

`ks = 0` 时 `F = −D·v`：

- 静止 `v=0` → `F=0`，手臂停在放手的位置，和原浮动一致；
- 运动时仍有方向性阻力，且 D 的各向异性是 15:3 = **5:1**；
- 阻尼还在，松手不会来回荡。

**如果连 D 一起缩**：`ks=0` 就变成完全无力矩，既没有阻力，也没有阻尼，
和"关掉 assist"没区别，各向异性也就没了。这是为什么单独加一个只乘 K 的系数。

### 10.3 为什么不做"参考点缓慢跟随"

把 `p_ref` 慢慢移向 `p`（leaky reference / lead-through）也能让手臂最终停住，
但那是第三种交互：推的时候有阻力，停住后力会随时间衰减。
它既不是"保持位置"（力会消失），也不是"随动随停"（需要等几秒），
实现和调参都更复杂。当前用 `k_scale` 二选一，语义清楚，演示时也好解释。

---

## 11. 实机现象的对应解释

| 现象 | 原理 | 结论 |
| --- | --- | --- |
| 水平推时竖直下沉 45 mm | `e = K⁻¹F_hand`，手带 1.19 N 竖直分力，`KZ=30` | 不是耦合 bug；盯 `F[2]`，把推力调水平 |
| 抬臂时 J4 猛转 | 位置阻抗不约束姿态，`τ4 = J[2][4]·F[2]`，腕关节最轻 | 正常；需要时开 6D 项 |
| 推不到 6 N | 静态 `F` 等于手力，K 只定位移 | 看 `−F/e`，不必强求饱和 |
| `F` 没停在 6.0、`sat_cnt` 涨 | 范数限幅 + 快速推进的阻尼瞬态 | 拆成 `sat_force_cnt` / `sat_tau_cnt` 看 |
| 大振幅抖动 | 限幅削阻尼 + 23 ms 速度滞后 | 降 `scale`；`sat_tau_cnt` 不涨就只是手感 |
| 回 `p_ref` 轻微抖动 | 回程 `\|F\|` 顶到 6 N，阻尼被同比裁剪 | `k_scale=0` 或降 `scale` |
| 松手回弹，不能随动随停 | 弹簧的定义 | `k_scale=0` |

---

## 12. 怎么验证这些代码是对的

`tools/test_cartesian_assist.py` 用 gcc 编译纯计算部分（`CART_ASSIST_HOST_TEST`），
不碰硬件，检查：

1. **FK**：home 的 TCP 与离线计算一致（0.0821, −0.0099, 0.1018 m）；
2. **雅可比**：解析 `J` 对 FK 做**中心差分**（h=10⁻³），误差 < 2×10⁻⁴。
   差分只用到正运动学，与解析式独立，所以能抓符号、索引、frames 取错；
3. **参考点/软启动**：第一次使能 `gain=0`、`e=0`，500 ms 后 `gain=1`；
4. **符号与映射**：`F` 与 `e` 反号，`τ = JᵀF`，逐关节核对 `torque_sign`；
5. **缩放**：`scale=0.5` 力减半，负值钳到 0；
6. **限幅**：大位移时 `|F|=6.00`、方向不变、计数增加；
7. **6D**：`Jw` 等于关节轴；绕 J5 转 0.05 rad 时 `e_R=0.05·z5`、
   `M=−Kr·e_R`；1 rad 时 `|M|` 裁到 0.30；
8. **`k_scale=0`**：有位移但静止 → `F=0`；匀速 → `F=−D·v`，无 `K·e` 项；
9. **包装层**：关闭/非浮动/熔断时不改 `tor_set`，重力快照正确。

实机验收则用"等位移看力"：两个姿态下水平 120 N/m、竖直 30 N/m，比值 4:1，
详见实测记录。

---

## 13. 参数为什么取这些值

| 参数 | 值 | 原因 |
| --- | --- | --- |
| `KX/KY` | 120 N/m | 25 mm 约 3 N，手感明确；50 mm 到 6 N |
| `KZ` | 30 N/m | 与水平成 4:1，作业要求的"竖直小阻力" |
| `DX/DY` | 15 N·s/m | `D/K≈0.125 s`，慢速拖动不发涩，松手不荡 |
| `DZ` | 3 N·s/m | 与 KZ 同比例，保持阻尼比 |
| `FMAX` | 6 N | home 下 J0 力矩 0.49 N·m，占 0.6 的 82% |
| `MMAX` | 0.3 N·m | J0/J5 上限 0.6 的 50% |
| `RAMP_MS` | 500 | 使能不拽手；再长会显得"没反应" |
| `VEL_ALPHA` | 0.35 | 23 ms 滞后，压噪声且不引入明显相位滞后 |
| `KRX/KRY` | 2.0 N·m/rad | 0.15 rad（8.6°）到 0.3 N·m 限幅 |
| `KRZ` | 1.0 N·m/rad | 绕 z 先软一点，避免 J0 过早饱和 |
| `DR` | 0.30 / 0.30 / 0.15 | 与 Kr 同比例 |

---

## 14. 局限与后续

1. **力矩级、无力矩前馈之外的动力学补偿**：没有惯性/科氏项，快速拖动时
   仍要靠人手和阻尼；100 Hz 带宽也限制了刚度上限。
2. **饱和边界的手感**：`FMAX` 同时削弹簧和阻尼，回程会略欠阻尼；
   `k_scale=0` 或降低 `scale` 可绕开。
3. **姿态项的线性化**：`e_R = sinθ·n`，只适合小角度；大角度回零需要
   对数映射或分段处理。
4. **零空间**：6 关节做 6D 任务，奇异位形附近仍可能有关节漂移；
   需要时加零空间手腕保持。
5. **TCP/工具**：装工具后必须量 `kCartAssistTcp`，否则力的作用点不对。
6. **实机定量**：6D 的 `e_R/M` 符号与幅值、`k_scale=0` 的实际手感，
   仍需下次上机逐项记录。
