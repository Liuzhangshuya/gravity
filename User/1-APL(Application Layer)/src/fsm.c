#include "dm4310_ctrl.h"
#include "dm4310_drv.h"

#include "gravity.h"
#include "fsm.h"
#include "math.h"
#include "button_status_read.h" 
#include "motor_protect.h"
#include "motor_function.h"


float q[6];
float arm[6];
float q_ikine[6];


static void motor_par_send(void);
static void fsm_param_get(fsm_t *fsm);

void fsm_run(fsm_t* fsm) {
	


    switch (fsm->state)
    {
        case wait_switch:
           
            motor_wait_feedback_update();
            if ( 1) {
                motor_pos_init_reset();
                fsm->state = fsm_pos_init;
            }
            break;

        case fsm_pos_init:
            if (motor_pos_init()) { 
                lock_flag = 0; 
                fsm->state = fsm_judge;
            }
            motor_par_send();
            break;

        case fsm_judge:
            break;

        case fsm_lock:			
            break;
        
        case fsm_geforce_off:
                motor_par_send();
            break;


        default:
					fsm->state = wait_switch;
            break;
    }
}
void fsm_run_test(fsm_t* fsm) {
    static uint8_t is_fsm_started = 0;
    static uint8_t lock_pin7_state_inited = 0;
    static GPIO_PinState lock_pin7_last_state = GPIO_PIN_RESET;
    GPIO_PinState current_pin7_state = GPIO_PIN_RESET;
    fsm_param_get(fsm); 

    if ((fsm->state != fsm_lock) && (fsm->state != fsm_test)) {
        lock_pin7_state_inited = 0;
    }

    if (is_fsm_started == 0) {
        fsm->state = wait_switch; 
        is_fsm_started = 1;
    }

    // 记录进入保护前状态机所处的状态，便于判断是回零中还是回零后掉臂触发。
    if (fsm->state != fsm_protect) {
        global_halt_fsm_state = (uint8_t)fsm->state;
    }

    if (global_system_halt_flag ||
        (fsm->state != wait_switch && !motor_feedback_ready())) {
        if (!motor_feedback_ready() && global_halt_reason == 0) {
            global_halt_reason = 5;
        }
        global_system_halt_flag = 1;
        fsm->state = fsm_protect;
        motor_par_send();
        return;
    }

    switch (fsm->state)
    {
        case wait_switch:
            
            motor_wait_feedback_update();
            if (lock_button_enable() == 1 && motor_feedback_ready()) {
                motor_pos_init_reset();
                fsm->state = fsm_pos_init;
            }
            break;

            case fsm_pos_init:
            if (motor_pos_init()) { 
                lock_flag = 0; 

                fsm->state = fsm_judge;
            }
            motor_par_send();
            break;

            case fsm_judge:
                
                if(lock_button_judge() == 1) {
                    fsm->state = fsm_lock;
                } else {
                    fsm->state = fsm_geforce_off;
                }
                break;

            case fsm_lock:
                if (lock_button_judge() == 0) {
                    lock_flag = 0; 
                    fsm->state = fsm_geforce_off;
                    break; 
                }

                current_pin7_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
                if (lock_pin7_state_inited == 0) {
                    lock_pin7_last_state = current_pin7_state;
                    lock_pin7_state_inited = 1;
                }

                if ((lock_pin7_last_state == GPIO_PIN_RESET) &&
                    (current_pin7_state == GPIO_PIN_SET)) {
                    lock_pin7_last_state = current_pin7_state;
                    motor_pos_run_reset();
                    fsm->state = fsm_test;
                    break; 
                }

                lock_pin7_last_state = current_pin7_state;
                
                motor_lock();
                motor_par_send();			
                break;

            case fsm_geforce_off:
                if (lock_button_judge() == 1) {
                    fsm->state = fsm_lock;
                    break;
                }
                /* 自由拖动：kp=0，只保留重力前馈和阻尼。
                 * 手推时几乎只有阻尼力，松手后手臂停下；重力补偿增益接近 1 时
                 * 残余重力很小，可以停在放开的位置，不需要再切到 lock。
                 * 增益还低时（如 0.6）仍会有部分重力，手臂会缓慢下沉。 */
                for(int i = 0; i < motor_num; i++) {
                    motor[i].cmd.pos_set = motor[i].para.pos;
                    motor[i].cmd.vel_set = 0;
                    motor[i].cmd.kp_set = 0;
                    motor[i].cmd.kd_set = MOTOR_FLOAT_KD;
                }
                motor_par_send();
                break;
            case fsm_test:
                if (motor_pos_run())
                {
                    lock_flag = 0;
                    fsm->state = fsm_lock;
                }
                motor_par_send();
                break;
            default:
                global_system_halt_flag = 1;
                motor_par_send();
                break;

    }
}

static void motor_par_send(void)
{
    
    static uint8_t compensation_started = 0;
    static uint32_t compensation_start_ms;
    if (!global_system_halt_flag) {
        if (!compensation_started) {
            compensation_started = 1;
            compensation_start_ms = HAL_GetTick();
        }
        Gravity_Arm_Run();
        float ramp = fminf((float)(HAL_GetTick() - compensation_start_ms) / 500.0f, 1.0f);
        for (int i = 0; i < motor_num; i++) {
            motor[i].cmd.tor_set *= ramp;
        }
    }
    motor_protect_run();
    calib_capture_run();
    ctrl_set();
    ctrl_send();
}


void fsm_param_get(fsm_t *fsm)
{
	static uint8_t i;
	Switch_Callback(fsm);

	q[0] = motor[0].para.pos;
	q[1] = motor[1].para.pos;
	q[2] = motor[2].para.pos;
	q[3] = motor[3].para.pos;
	q[4] = motor[4].para.pos;
	q[5] = motor[5].para.pos;

	for(i=0;i<motor_num;i++)
	{
		fsm->to_manipulator_data.param.motor[i].num=motor[i].para.pos;
	}
}
