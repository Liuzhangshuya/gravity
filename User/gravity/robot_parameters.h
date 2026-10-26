#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

/* 在此填入实测参数。长度 m，角度 rad，质量 kg。J0 为第一个电机。 */
/* 全部填完并核对后改为 1；不要用零值冒充未测量参数。 */
#define ROBOT_PARAMETERS_READY 1
#define GRAVITY_COMP_GAIN 1.0f

/* J0：LINK=-1 为底座，0~5 为所随动的连杆。MASS 必须实测/确认。 */
#define J0_A                                 0.0f // DH连杆长度，沿X轴，单位m
#define J0_ALPHA                             0.0f// DH连杆扭转角，绕X轴，单位rad
#define J0_D                                 0.042f// DH连杆偏距，沿Z轴，单位m
#define J0_THETA_OFFSET                      0.0f// DH固定角偏置，绕Z轴，单位rad
#define J0_ENCODER_ZERO                      0.3f// 参考姿态的编码器读数，单位rad
#define J0_POSITION_SIGN                     1// 编码器到模型的角度方向，填+1或-1
#define J0_TORQUE_SIGN                       1// 模型到电机的力矩方向，填+1或-1
#define J0_MASS                              0.32f// 第一个电机的质量，单位kg
#define J0_LINK                              (-1)// 质量所属刚体：-1底座，0~5关节后的连杆
#define J0_COM_X                             -0.00006f// 质心在所属连杆坐标系的X坐标，单位m
#define J0_COM_Y                             0.0f// 质心在所属连杆坐标系的Y坐标，单位m
#define J0_COM_Z                             -0.03446f// 质心在所属连杆坐标系的Z坐标，单位m

#define J1_A                                 0.0f
#define J1_ALPHA                             -1.570796f
#define J1_D                                 -0.022f
#define J1_THETA_OFFSET                      -1.583297f
#define J1_ENCODER_ZERO                      -1.5f
/* 实测标定（回零姿态 + J1/J2 同转姿态）：该电机安装方向与模型正方向相反，
 * 位置和力矩两个符号必须同时取 -1，否则 J1 前馈方向相反。 */
#define J1_POSITION_SIGN                     (-1)
#define J1_TORQUE_SIGN                       (-1)
#define J1_MASS                              0.32f
#define J1_LINK                              0
#define J1_COM_X                             -0.00006f
#define J1_COM_Y                             -0.01046f
#define J1_COM_Z                             -0.00001f

#define J2_A                                 0.120f
#define J2_ALPHA                             -3.141593f
#define J2_D                                 -0.0293f
#define J2_THETA_OFFSET                      -0.012500f
#define J2_ENCODER_ZERO                      -1.65f
/* 实测标定同 J1：位置和力矩符号同时取 -1。 */
#define J2_POSITION_SIGN                     (-1)
#define J2_TORQUE_SIGN                       (-1)
/* J2~J5 的 MASS 为等效质量（电机+连杆），由六组锁定姿态的力矩拟合得到；
 * 只填电机 0.32 kg 会让 J1/J2 的负载明显偏小。 */
#define J2_MASS                              0.44f
#define J2_LINK                              1
#define J2_COM_X                             0.11994f
#define J2_COM_Y                             -0.00002f
#define J2_COM_Z                             0.02946f

#define J3_A                                 0.083f
#define J3_ALPHA                             -1.570796f
#define J3_D                                 -0.111f
#define J3_THETA_OFFSET                      0.0f
#define J3_ENCODER_ZERO                      -0.2f
#define J3_POSITION_SIGN                     1
#define J3_TORQUE_SIGN                       1
#define J3_MASS                              0.44f
#define J3_LINK                              2
#define J3_COM_X                             0.08294f
#define J3_COM_Y                             -0.01146f
#define J3_COM_Z                             -0.00001f

#define J4_A                                 0.0f
#define J4_ALPHA                             -1.570796f
#define J4_D                                 0.0075f
#define J4_THETA_OFFSET                      3.141593f
#define J4_ENCODER_ZERO                      0.01f
#define J4_POSITION_SIGN                     1
#define J4_TORQUE_SIGN                       1
#define J4_MASS                              0.44f
#define J4_LINK                              3
#define J4_COM_X                             -0.00006f
#define J4_COM_Y                             -0.01046f
#define J4_COM_Z                             -0.00001f

#define J5_A                                 0.0f              
#define J5_ALPHA                             -1.570796f
#define J5_D                                 -0.080f
#define J5_THETA_OFFSET                      -1.570796f
#define J5_ENCODER_ZERO                      0.0f
#define J5_POSITION_SIGN                     1
#define J5_TORQUE_SIGN                       1
#define J5_MASS                              0.44f
#define J5_LINK                              4
#define J5_COM_X                             -0.00006f
/* J4 单独旋转标定：J4 力矩随 J4 角度几乎不变（1.185 rad 只变约 0.005 N·m），
 * 等效质心必须靠近 J4 轴。原值 -0.06846 会让 J4 前馈大约 5 倍。 */
#define J5_COM_Y                             -0.008f
#define J5_COM_Z                             -0.00002f

#endif