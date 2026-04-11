/**
 * @file     user_function.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the functions for the board level support package.
 *
 * @attention
 *
 * THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 * CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 * TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 * HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 * CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 *
 * <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
 */

/** Define to prevent recursive inclusion */
#define _USER_FUNCTION_C_

/** Files includes */
#include "user_function.h"
#include "parameter.h"
#include "pwm_gen.h"
#include "HallHandle.h"
#include "PID.h"
#include "FOC_Math.h"
#include "Diagnose.h"

/*------------------- Private variables ---------------*/
int32_t IsumGain;
int32_t IshuntGain;
int32_t SpeedGain;
uint32_t VbusGain;
ADC_components ADC_Structure;

uint8_t MotionModeStatus = MOTION_MODE_STOP;
uint8_t MotionRunEnable = 0U;
int8_t MotionRunDir = MOTOR_DIR;
uint16_t MotionSpeedCmdRpm10 = 0U;
uint8_t MotionPositionEnable = 0U;
int16_t MotionPositionTargetQ15 = 0;
int16_t MotionPositionHoldIqQ15 = POSITION_HOLD_IQ_Q15;
int16_t MotionSpeedCurrentLimitQ15 = SPEED_LOOP_MAX_CURRENT_Q15;
uint16_t MotionTorquePermille = 0U;
uint8_t MotionParkAngleValid = 0U;
int16_t MotionParkAngleQ15 = 0;

#define MOTION_DATA_FLASH_ADDR          0x1FE00000UL
#define MOTION_DATA_FLASH_PAGE          0x1FE00000UL
#define MOTION_DATA_MAGIC               0x4D544E31UL
#define MOTION_JOG_STEP_Q15             2730
#define MOTION_JOG_REACH_Q15            900
#define MOTION_JOG_REPEAT_MS            120U
#define MOTION_JOG_SPEED_RPM10          180U
#define MOTION_REVERSE_STOP_RPM10       60U
#define MOTION_SAVE_READY_RPM10         20U
#define MOTION_RECIP_MIN_PERIOD_MS      300U
#define MOTION_RECIP_MAX_PERIOD_MS      1000U
#define MOTION_RUN_MIN_SPEED_RPM        100U
#define MOTION_RUN_MAX_SPEED_RPM        12000U
#define MOTION_RECIP_MAX_SPEED_RPM      5000U
#define MOTION_RECIP_MIN_TURNS          1U
#define MOTION_RECIP_MAX_TURNS          8U
#define MOTION_TORQUE_FILTER_SHIFT      2U
#define MOTION_TORQUE_FAULT_TICKS       10U
#define MOTION_TORQUE_RATED_Q15         ((ISHUNT_GAIN * MOTOR_NOMINAL_CURRENT_A10) / 10)
#define MOTION_TORQUE_HEAVY_Q15         ((ISHUNT_GAIN * MOTOR_NOMINAL_CURRENT_A10 * 12) / 100)
#define MOTION_TORQUE_TRIP_Q15          ((ISHUNT_GAIN * MOTOR_NOMINAL_CURRENT_A10 * 16) / 100)
#define MOTION_SPEED_FLOOR_RPM10        10U
#define MOTION_HALL_STEPS_PER_TURN      6UL

typedef struct
{
    uint32_t magic;
    int32_t park_angle_q15;
    uint32_t checksum;
} MotionPersistRecord;

typedef struct
{
    uint8_t requested_mode;
    uint16_t configured_speed_rpm;
    uint16_t configured_param;
    int8_t active_dir;
    int8_t recip_dir;
    uint16_t recip_elapsed_ms;
    uint32_t recip_start_steps;
    uint8_t jog_continuous;
    int8_t jog_dir;
    uint16_t jog_repeat_ms;
    uint8_t save_pending;
    uint16_t torque_filt_q15;
    uint8_t over_torque_ticks;
} MotionCtrlState;

static MotionCtrlState s_motion = {0};

/*------------------ Private functions ----------------*/
static uint16_t Motion_Abs16(int16_t value);
static uint16_t Motion_FrameChecksum(const uint8_t *buf, uint16_t len);
static int16_t Motion_WrapAngleQ15(int32_t angle);
static int16_t Motion_AngleDiffQ15(int16_t target, int16_t angle);
static uint16_t Motion_RpmToRpm10(uint16_t rpm);
static int8_t Motion_DecodeDir(uint8_t dir_code);
static uint32_t Motion_RecordChecksum(const MotionPersistRecord *record);
static void Motion_LoadPersist(void);
static void Motion_SavePersist(void);
static void Motion_ResetDynamicState(uint8_t new_mode);
static void Motion_StopAll(void);
static void Motion_BuildReply(uint8_t status, uint8_t *tx, uint16_t tx_cap);

static uint16_t Motion_Abs16(int16_t value)
{
    return (value >= 0) ? (uint16_t)value : (uint16_t)(-value);
}

static uint16_t Motion_FrameChecksum(const uint8_t *buf, uint16_t len)
{
    uint16_t sum = 0U;
    uint16_t i;

    for (i = 0U; i < len; i++)
    {
        sum = (uint16_t)(sum + buf[i]);
    }
    return sum;
}

static int16_t Motion_WrapAngleQ15(int32_t angle)
{
    while (angle > 32767L)
    {
        angle -= 65536L;
    }
    while (angle < -32768L)
    {
        angle += 65536L;
    }
    return (int16_t)angle;
}

static int16_t Motion_AngleDiffQ15(int16_t target, int16_t angle)
{
    int32_t diff = (int32_t)target - (int32_t)angle;

    if (diff > 32767L)
    {
        diff -= 65536L;
    }
    else if (diff < -32768L)
    {
        diff += 65536L;
    }
    return (int16_t)diff;
}

static uint16_t Motion_RpmToRpm10(uint16_t rpm)
{
    return (uint16_t)((rpm + 5U) / 10U);
}

static int8_t Motion_DecodeDir(uint8_t dir_code)
{
    if (dir_code == MOTION_DIR_FORWARD)
    {
        return MOTOR_DIR;
    }
    if (dir_code == MOTION_DIR_REVERSE)
    {
        return (int8_t)(-MOTOR_DIR);
    }
    return 0;
}

static uint32_t Motion_RecordChecksum(const MotionPersistRecord *record)
{
    return record->magic ^ (uint32_t)record->park_angle_q15 ^ 0x13572468UL;
}

static void Motion_LoadPersist(void)
{
    const MotionPersistRecord *record = (const MotionPersistRecord *)MOTION_DATA_FLASH_ADDR;

    if ((record->magic == MOTION_DATA_MAGIC) &&
        (record->checksum == Motion_RecordChecksum(record)) &&
        (record->park_angle_q15 <= 32767L) &&
        (record->park_angle_q15 >= -32768L))
    {
        MotionParkAngleQ15 = (int16_t)record->park_angle_q15;
        MotionParkAngleValid = 1U;
    }
    else
    {
        MotionParkAngleQ15 = 0;
        MotionParkAngleValid = 1U;
    }
}

static void Motion_SavePersist(void)
{
    MotionPersistRecord record;
    FLASH_Status flash_status;

    if ((s_motion.save_pending == 0U) || (MotionParkAngleValid == 0U))
    {
        return;
    }

    record.magic = MOTION_DATA_MAGIC;
    record.park_angle_q15 = MotionParkAngleQ15;
    record.checksum = Motion_RecordChecksum(&record);

    FLASH_Unlock();
    FLASH_ClearFlag((uint16_t)(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR));

    flash_status = FLASH_ErasePage(MOTION_DATA_FLASH_PAGE);
    if (flash_status == FLASH_COMPLETE)
    {
        flash_status = FLASH_ProgramWord(MOTION_DATA_FLASH_ADDR + 0U, record.magic);
    }
    if (flash_status == FLASH_COMPLETE)
    {
        flash_status = FLASH_ProgramWord(MOTION_DATA_FLASH_ADDR + 4U, (uint32_t)record.park_angle_q15);
    }
    if (flash_status == FLASH_COMPLETE)
    {
        flash_status = FLASH_ProgramWord(MOTION_DATA_FLASH_ADDR + 8U, record.checksum);
    }

    FLASH_Lock();

    if (flash_status == FLASH_COMPLETE)
    {
        s_motion.save_pending = 0U;
    }
}

static void Motion_ResetDynamicState(uint8_t new_mode)
{
    s_motion.requested_mode = new_mode;
    s_motion.recip_elapsed_ms = 0U;
    s_motion.recip_start_steps = g_hall_transition_total;
    s_motion.jog_repeat_ms = 0U;

    if (new_mode == MOTION_MODE_RECIP_TIME || new_mode == MOTION_MODE_RECIP_TURNS)
    {
        s_motion.recip_dir = MOTOR_DIR;
    }
    else if (new_mode == MOTION_MODE_REVERSE)
    {
        s_motion.recip_dir = (int8_t)(-MOTOR_DIR);
    }
    else
    {
        s_motion.recip_dir = MOTOR_DIR;
    }
}

static void Motion_StopAll(void)
{
    MotionModeStatus = MOTION_MODE_STOP;
    MotionRunEnable = 0U;
    MotionSpeedCmdRpm10 = 0U;
    MotionPositionEnable = 0U;
    MotionPositionTargetQ15 = MotionParkAngleQ15;
    MotionPositionHoldIqQ15 = POSITION_HOLD_IQ_Q15;
}

static void Motion_BuildReply(uint8_t status, uint8_t *tx, uint16_t tx_cap)
{
    uint16_t checksum;
    uint16_t speed_rpm;

    if ((tx == NULL) || (tx_cap < MOTION_UART_FRAME_LEN))
    {
        return;
    }

    speed_rpm = (uint16_t)((HALL1.SpeedTemp >= 0) ? HALL1.SpeedTemp : -HALL1.SpeedTemp);
    speed_rpm = (uint16_t)(speed_rpm * 10U);

    tx[0] = MOTION_UART_SYNC_RSP;
    tx[1] = status;
    tx[2] = MotionModeStatus;
    tx[3] = FAULT.Byte;
    tx[4] = (uint8_t)(speed_rpm & 0xFFU);
    tx[5] = (uint8_t)(speed_rpm >> 8);
    tx[6] = (uint8_t)(MotionTorquePermille & 0xFFU);
    tx[7] = (uint8_t)(MotionTorquePermille >> 8);

    checksum = Motion_FrameChecksum(tx, 8U);
    tx[8] = (uint8_t)(checksum & 0xFFU);
    tx[9] = (uint8_t)(checksum >> 8);
}

/****************************************************************
	函数名：Init_Parameter
	描述：上电初始化变量						
	输入：无
	输出：无
****************************************************************/
void Init_Parameter(void)
{
	IshuntGain = ISHUNT_GAIN/10;								//unit 0.1A					
	IsumGain = 100*4096/ISUM_GAIN;								//unit 0.01A
	VbusGain = (122*(VBUS_PULL_UP_R + VBUS_PULL_DOWN_R)*1024)/(1000 * VBUS_PULL_DOWN_R); //unit 0.01V
	SpeedGain = 6000000/POLEPAIRS;								//RPM
	
	InitPI();
	InitNormalization(POSITION_POT_MIN_ADC, POSITION_POT_MAX_ADC, MOTOR_COMMAND_MAX_RPM10, &RP);
	LoopCmp_Init();
	HALLModuleInit(&HALL1);
	PWM_GEN_init(&pwm_gen);
    MotionCtrl_Init();
}

void MotionCtrl_Init(void)
{
    Motion_LoadPersist();

    s_motion.configured_speed_rpm = MOTION_RUN_MIN_SPEED_RPM;
    s_motion.configured_param = MOTION_RECIP_MIN_PERIOD_MS;
    s_motion.active_dir = MOTOR_DIR;
    s_motion.recip_dir = MOTOR_DIR;
    s_motion.requested_mode = MOTION_MODE_STOP;
    s_motion.jog_continuous = 0U;
    s_motion.jog_dir = MOTOR_DIR;
    s_motion.save_pending = 0U;
    s_motion.torque_filt_q15 = 0U;
    s_motion.over_torque_ticks = 0U;

    MotionRunDir = MOTOR_DIR;
    MotionPositionTargetQ15 = MotionParkAngleQ15;
    MotionPositionHoldIqQ15 = POSITION_HOLD_IQ_Q15;
    MotionSpeedCurrentLimitQ15 = SPEED_LOOP_MAX_CURRENT_Q15;
    Motion_StopAll();
}

uint16_t MotionCtrl_ProcessFrame(const uint8_t *rx, uint16_t rx_len, uint8_t *tx, uint16_t tx_cap)
{
    uint16_t checksum_rx;
    uint16_t checksum_calc;
    uint16_t speed_rpm;
    uint16_t param;
    uint8_t mode;
    uint8_t action;
    int8_t dir;
    uint8_t status = MOTION_STATUS_OK;

    if ((rx == NULL) || (rx_len < MOTION_UART_FRAME_LEN))
    {
        return 0U;
    }

    if (rx[0] != MOTION_UART_SYNC_REQ)
    {
        status = MOTION_STATUS_BAD_SYNC;
        Motion_BuildReply(status, tx, tx_cap);
        return MOTION_UART_FRAME_LEN;
    }

    checksum_rx = (uint16_t)rx[8] | ((uint16_t)rx[9] << 8);
    checksum_calc = Motion_FrameChecksum(rx, 8U);
    if (checksum_rx != checksum_calc)
    {
        status = MOTION_STATUS_BAD_CHECKSUM;
        Motion_BuildReply(status, tx, tx_cap);
        return MOTION_UART_FRAME_LEN;
    }

    switch (rx[1])
    {
    case MOTION_CMD_SET_MODE:
        mode = rx[2];
        speed_rpm = (uint16_t)rx[4] | ((uint16_t)rx[5] << 8);
        param = (uint16_t)rx[6] | ((uint16_t)rx[7] << 8);

        if (mode == MOTION_MODE_STOP)
        {
            Motion_ResetDynamicState(MOTION_MODE_STOP);
            s_motion.jog_continuous = 0U;
        }
        else if ((mode == MOTION_MODE_FORWARD) || (mode == MOTION_MODE_REVERSE))
        {
            if ((speed_rpm < MOTION_RUN_MIN_SPEED_RPM) || (speed_rpm > MOTION_RUN_MAX_SPEED_RPM))
            {
                status = MOTION_STATUS_BAD_PARAM;
                break;
            }

            s_motion.configured_speed_rpm = speed_rpm;
            s_motion.configured_param = 0U;
            s_motion.jog_continuous = 0U;
            Motion_ResetDynamicState(mode);
        }
        else if (mode == MOTION_MODE_RECIP_TIME)
        {
            if ((speed_rpm < MOTION_RUN_MIN_SPEED_RPM) || (speed_rpm > MOTION_RECIP_MAX_SPEED_RPM) ||
                (param < MOTION_RECIP_MIN_PERIOD_MS) || (param > MOTION_RECIP_MAX_PERIOD_MS))
            {
                status = MOTION_STATUS_BAD_PARAM;
                break;
            }

            s_motion.configured_speed_rpm = speed_rpm;
            s_motion.configured_param = param;
            s_motion.jog_continuous = 0U;
            Motion_ResetDynamicState(mode);
        }
        else if (mode == MOTION_MODE_RECIP_TURNS)
        {
            if ((speed_rpm < MOTION_RUN_MIN_SPEED_RPM) || (speed_rpm > MOTION_RECIP_MAX_SPEED_RPM) ||
                (param < MOTION_RECIP_MIN_TURNS) || (param > MOTION_RECIP_MAX_TURNS))
            {
                status = MOTION_STATUS_BAD_PARAM;
                break;
            }

            s_motion.configured_speed_rpm = speed_rpm;
            s_motion.configured_param = param;
            s_motion.jog_continuous = 0U;
            Motion_ResetDynamicState(mode);
        }
        else
        {
            status = MOTION_STATUS_BAD_PARAM;
        }
        break;

    case MOTION_CMD_JOG:
        action = rx[2];
        dir = Motion_DecodeDir(rx[3]);
        if (dir == 0)
        {
            status = MOTION_STATUS_BAD_PARAM;
            break;
        }

        if (action == 0U)
        {
            MotionParkAngleQ15 = Motion_WrapAngleQ15((int32_t)MotionParkAngleQ15 + (int32_t)dir * MOTION_JOG_STEP_Q15);
            MotionParkAngleValid = 1U;
            s_motion.jog_continuous = 0U;
            s_motion.jog_dir = dir;
            Motion_ResetDynamicState(MOTION_MODE_JOG);
        }
        else if (action == 1U)
        {
            if ((s_motion.requested_mode != MOTION_MODE_JOG) || (s_motion.jog_continuous == 0U) || (s_motion.jog_dir != dir))
            {
                MotionParkAngleQ15 = Motion_WrapAngleQ15((int32_t)MotionParkAngleQ15 + (int32_t)dir * MOTION_JOG_STEP_Q15);
                MotionParkAngleValid = 1U;
            }
            s_motion.jog_continuous = 1U;
            s_motion.jog_dir = dir;
            Motion_ResetDynamicState(MOTION_MODE_JOG);
        }
        else if (action == 2U)
        {
            s_motion.jog_continuous = 0U;
            s_motion.save_pending = 1U;
            Motion_ResetDynamicState(MOTION_MODE_STOP);
        }
        else
        {
            status = MOTION_STATUS_BAD_PARAM;
        }
        break;

    case MOTION_CMD_STOP:
        s_motion.jog_continuous = 0U;
        Motion_ResetDynamicState(MOTION_MODE_STOP);
        break;

    case MOTION_CMD_QUERY:
        break;

    default:
        status = MOTION_STATUS_BAD_CMD;
        break;
    }

    MotionModeStatus = s_motion.requested_mode;
    Motion_BuildReply(status, tx, tx_cap);
    return MOTION_UART_FRAME_LEN;
}

void MotionCtrl_Update5ms(int16_t angle_q15, int16_t speed_rpm10, int16_t iq_q15)
{
    uint16_t abs_iq_q15;
    uint16_t base_speed_rpm10;
    uint16_t derated_speed_rpm10;
    uint16_t heavy_th_q15 = MOTION_TORQUE_HEAVY_Q15;
    uint16_t trip_th_q15 = MOTION_TORQUE_TRIP_Q15;
    int16_t angle_err_q15;
    int8_t desired_dir = MOTOR_DIR;
    uint32_t step_span;

    abs_iq_q15 = Motion_Abs16(iq_q15);
    s_motion.torque_filt_q15 = (uint16_t)(s_motion.torque_filt_q15 +
        (((int32_t)abs_iq_q15 - (int32_t)s_motion.torque_filt_q15) >> MOTION_TORQUE_FILTER_SHIFT));

    if (MOTION_TORQUE_RATED_Q15 > 0)
    {
        MotionTorquePermille = (uint16_t)(((uint32_t)s_motion.torque_filt_q15 * 1000U) / MOTION_TORQUE_RATED_Q15);
    }
    else
    {
        MotionTorquePermille = 0U;
    }

    if (s_motion.torque_filt_q15 >= trip_th_q15)
    {
        if (s_motion.over_torque_ticks < 250U)
        {
            s_motion.over_torque_ticks++;
        }
    }
    else
    {
        s_motion.over_torque_ticks = 0U;
    }

    if (s_motion.over_torque_ticks >= MOTION_TORQUE_FAULT_TICKS)
    {
        FAULT.bit.LockedFlag = 1U;
        s_motion.jog_continuous = 0U;
        Motion_ResetDynamicState(MOTION_MODE_STOP);
    }

    MotionRunDir = s_motion.active_dir;
    MotionSpeedCurrentLimitQ15 = SPEED_LOOP_MAX_CURRENT_Q15;
    MotionPositionHoldIqQ15 = POSITION_HOLD_IQ_Q15;
    MotionPositionTargetQ15 = MotionParkAngleQ15;
    MotionModeStatus = s_motion.requested_mode;

    if (s_motion.requested_mode == MOTION_MODE_STOP)
    {
        Motion_StopAll();
        if ((s_motion.save_pending != 0U) && (Motion_Abs16(speed_rpm10) <= MOTION_SAVE_READY_RPM10))
        {
            Motion_SavePersist();
        }
        return;
    }

    if (FAULT.Byte != 0U)
    {
        Motion_StopAll();
        return;
    }

    base_speed_rpm10 = Motion_RpmToRpm10(s_motion.configured_speed_rpm);
    derated_speed_rpm10 = base_speed_rpm10;

    if ((trip_th_q15 > heavy_th_q15) && (s_motion.torque_filt_q15 > heavy_th_q15))
    {
        uint32_t span_q15 = (uint32_t)(trip_th_q15 - heavy_th_q15);
        uint32_t over_q15 = (uint32_t)(s_motion.torque_filt_q15 - heavy_th_q15);
        uint32_t limit_span = (uint32_t)(MOTION_TORQUE_TRIP_Q15 - SPEED_LOOP_MAX_CURRENT_Q15);

        if (over_q15 > span_q15)
        {
            over_q15 = span_q15;
        }

        derated_speed_rpm10 = (uint16_t)(base_speed_rpm10 -
            (((uint32_t)(base_speed_rpm10 - MOTION_SPEED_FLOOR_RPM10) * over_q15) / span_q15));
        MotionSpeedCurrentLimitQ15 = (int16_t)(SPEED_LOOP_MAX_CURRENT_Q15 +
            (int16_t)((limit_span * over_q15) / span_q15));
    }

    if (s_motion.requested_mode == MOTION_MODE_FORWARD)
    {
        desired_dir = MOTOR_DIR;
        MotionPositionEnable = 0U;
    }
    else if (s_motion.requested_mode == MOTION_MODE_REVERSE)
    {
        desired_dir = (int8_t)(-MOTOR_DIR);
        MotionPositionEnable = 0U;
    }
    else if (s_motion.requested_mode == MOTION_MODE_RECIP_TIME)
    {
        MotionPositionEnable = 0U;
        s_motion.recip_elapsed_ms = (uint16_t)(s_motion.recip_elapsed_ms + 5U);
        if (s_motion.recip_elapsed_ms >= (uint16_t)(s_motion.configured_param / 2U))
        {
            s_motion.recip_elapsed_ms = 0U;
            s_motion.recip_dir = (int8_t)(-s_motion.recip_dir);
        }
        desired_dir = s_motion.recip_dir;
    }
    else if (s_motion.requested_mode == MOTION_MODE_RECIP_TURNS)
    {
        MotionPositionEnable = 0U;
        step_span = g_hall_transition_total - s_motion.recip_start_steps;
        if (step_span >= ((uint32_t)s_motion.configured_param * MOTION_HALL_STEPS_PER_TURN))
        {
            s_motion.recip_start_steps = g_hall_transition_total;
            s_motion.recip_dir = (int8_t)(-s_motion.recip_dir);
        }
        desired_dir = s_motion.recip_dir;
    }
    else
    {
        angle_err_q15 = Motion_AngleDiffQ15(MotionParkAngleQ15, angle_q15);
        desired_dir = (angle_err_q15 >= 0) ? MOTOR_DIR : (int8_t)(-MOTOR_DIR);
        MotionModeStatus = MOTION_MODE_JOG;
        MotionPositionEnable = 1U;
        MotionPositionTargetQ15 = MotionParkAngleQ15;
        derated_speed_rpm10 = MOTION_JOG_SPEED_RPM10;

        if ((s_motion.jog_continuous != 0U) && (Motion_Abs16(angle_err_q15) <= MOTION_JOG_REACH_Q15))
        {
            s_motion.jog_repeat_ms = (uint16_t)(s_motion.jog_repeat_ms + 5U);
            if (s_motion.jog_repeat_ms >= MOTION_JOG_REPEAT_MS)
            {
                s_motion.jog_repeat_ms = 0U;
                MotionParkAngleQ15 = Motion_WrapAngleQ15((int32_t)MotionParkAngleQ15 + (int32_t)s_motion.jog_dir * MOTION_JOG_STEP_Q15);
                MotionPositionTargetQ15 = MotionParkAngleQ15;
            }
        }
        else
        {
            s_motion.jog_repeat_ms = 0U;
        }

        if ((s_motion.jog_continuous == 0U) && (Motion_Abs16(angle_err_q15) <= MOTION_JOG_REACH_Q15))
        {
            s_motion.save_pending = 1U;
            Motion_ResetDynamicState(MOTION_MODE_STOP);
            Motion_StopAll();
            return;
        }
    }

    if ((s_motion.active_dir != desired_dir) && (Motion_Abs16(speed_rpm10) > MOTION_REVERSE_STOP_RPM10))
    {
        MotionRunEnable = 0U;
        MotionSpeedCmdRpm10 = 0U;
        return;
    }

    s_motion.active_dir = desired_dir;
    MotionRunDir = desired_dir;
    MotionRunEnable = 1U;
    MotionSpeedCmdRpm10 = derated_speed_rpm10;
}



