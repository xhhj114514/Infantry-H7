#include "LK_motor.h"
#include "motor_def.h"
#include "stdlib.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"

static uint8_t idx;
static LKMotorInstance *lkmotor_instance[LK_MOTOR_MX_CNT] = {NULL};

/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 * @note  State1 && State2  有需要其他需要自行增加
 */
static void LKDecode(CANInstance *_instance)
{
    static LKMotorInstance *motor;
    static LKMotor_Measure_t *measure;
    //Find Motor
    motor = (LKMotorInstance *)_instance->id; //CAN(Father)Ptr to Motor(Children),,todo?：优化结构体识别，省去自行记忆背诵解析
    measure = &motor->measure;
    uint8_t *rx_buff = _instance->rx_buff;

    DaemonReload(motor->daemon); // 喂狗
    measure->feed_dt = DWT_GetDeltaT(&measure->feed_dwt_cnt);
     
    //Update Last Mea
    measure->last_ecd = measure->ecd;
    switch(rx_buff[0])
    {
        case LK_READ_STATE_1:
        {
            if(rx_buff[7] & 0x80)motor->STATE = MOTORSIGNALLOST;
            else if(rx_buff[7] & 0x40)motor->STATE = MOTORSTALL;
            else if(rx_buff[7] & 0x20)motor->STATE = MOTORSHORTED;
            else if(rx_buff[7] & 0x10)motor->STATE = MOTOROVERCURRENT;
            else if(rx_buff[7] & 0x08)motor->STATE = MOTOROVERTEMP;
            else if(rx_buff[7] & 0x04)motor->STATE = DRIVEROVERTREMPPROTECT;
            else if(rx_buff[7] & 0x02)motor->STATE = HIGHVOLTAGEPROTECT;
            else if(rx_buff[7] & 0x01)motor->STATE = LOWVOLTAGEPROTECT;
            else motor->STATE = NORMAL;
        }
        break;
        case LK_READ_STATE_2:
        {
            motor->measure.temperature = rx_buff[1];

            motor->measure.real_current = (1 - CURRENT_SMOOTH_LPF) * measure->real_current +
                            CURRENT_SMOOTH_LPF * (float)((int16_t)(rx_buff[3]<<8 | rx_buff[2]))*LK_MF_RAW2CUR;

            motor->measure.speed_rads = (1 - SPEED_SMOOTH_LPF) * measure->speed_rads +
                          DEGREE_2_RAD * SPEED_SMOOTH_LPF * (float)((int16_t)(rx_buff[5]<<8 | rx_buff[4]))*LK_RAW2SPD;

            motor->measure.angle_single = (float)((int16_t)(rx_buff[7]<<8 | rx_buff[6]))*LK_ECD2ANGLE;
            if (measure->ecd - measure->last_ecd > 32768)
                measure->ACCrotation--;
            else if (measure->ecd - measure->last_ecd < -32768)
                measure->ACCrotation++;
            measure->ACCangle = measure->ACCrotation * 360 + measure->angle_single;
        }
        break;
        default:
        { 
        }
        break;
    }
}

static void LKLostCallback(void *motor_ptr)
{
    LKMotorInstance *motor = (LKMotorInstance *)motor_ptr;
}

LKMotorInstance *LKMotorInit(Motor_Init_Config_s *config)
{
    LKMotorInstance *motor = (LKMotorInstance *)malloc(sizeof(LKMotorInstance));
    motor = (LKMotorInstance *)malloc(sizeof(LKMotorInstance));
    memset(motor, 0, sizeof(LKMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.id = motor;
    config->can_init_config.can_module_callback = LKDecode;
    config->can_init_config.rx_id = LK_CAN_RXID_BASE + config->can_init_config.tx_id;
    config->can_init_config.tx_id = config->can_init_config.tx_id- 1 + LK_CAN_TXID_BASE ; 
    motor->motor_can_ins = CANRegister(&config->can_init_config);

    LKMotorEnable(motor);
    DWT_GetDeltaT(&motor->measure.feed_dwt_cnt);
    lkmotor_instance[idx++] = motor;

    Daemon_Init_Config_s daemon_config = {
        .callback = LKLostCallback,
        .owner_id = motor,
        .reload_count = 50, //ms
    };
    motor->daemon = DaemonRegister(&daemon_config);

    return motor;
}

void CheckMotor(LKMotorInstance *motor)
{   static uint8_t AA=0;
    if(AA<=1000)
    {
        motor->motor_can_ins->tx_buff[0] = LK_READ_STATE_2;
        if(AA % 1000 == 0)
        {
            motor->motor_can_ins->tx_buff[1] = LK_READ_STATE_1;
        }
    }
}

void LKMotorControl()
{
    static uint8_t II=0;
    for(II=0; II<LK_MOTOR_MX_CNT; II++)
    {
        CheckMotor(lkmotor_instance[II]);
        if(lkmotor_instance[II]->stop_flag == MOTOR_ENALBED) 
        {
            CANTransmit(lkmotor_instance[II]->motor_can_ins, 0.2);
        }
        else if (lkmotor_instance[II]->stop_flag == MOTOR_STOP)
        { 
            memset(lkmotor_instance[II]->motor_can_ins->tx_buff , 0, sizeof(uint8_t));
        }
    }
}

void LKMotorStop(LKMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void LKMotorEnable(LKMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void LKMotorSetRef(LKMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

uint8_t LKMotorIsOnline(LKMotorInstance *motor)
{
    return DaemonIsOnline(motor->daemon);
}
