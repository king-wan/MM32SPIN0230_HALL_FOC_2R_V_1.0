/*-------------------- Includes -----------------------*/
#include "drv_inc.h"
#include "PID.h"
#include "HallHandle.h"
#include "MC_Drive.h"
#include "FOC_Math.h"
#include "pwm_gen.h"
#include "user_function.h"
#include "Diagnose.h"
#include "parameter.h"
#include "hall_tune.h"

/*------------------- Private variables ---------------*/
TIMFlagType TIMFlag;

uint8_t MotorState = IDLESTATE;
uint16_t Time5msCnt = 0;
uint16_t Time200msCnt = 0;
int16_t ElecAngle = 0;
int16_t IdRef = -1000;
int16_t IqRef = 0;
int16_t AngleTest = 0;
Trig_Components Vector_Components;

uint32_t u32IuSum = 0;
uint32_t u32IvSum = 0;
int32_t s32IuOffsetQ3 = 0;
int32_t s32IvOffsetQ3 = 0;
int32_t s32IuFiltQ3 = 0;
int32_t s32IvFiltQ3 = 0;
uint8_t PositionHoldEnable = 0;
int16_t PositionHoldTargetAngle = 0;
int16_t PositionHoldIq = 0;

/* Fixed-angle start/stop position hold */
#define POS_HOLD_ENTRY_SPD          40
#define POS_HOLD_IQ_STOP            220
#define POS_HOLD_IQ_START           START_ALIGN_IQ_Q15
#define POS_HOLD_ANGLE_DEAD         900
#define START_ALIGN_PWM_CYCLES      1200
#define START_LOCK_PWM_CYCLES       900
#define START_KICK_IQ               START_KICK_IQ_Q15
#define START_KICK_ADV_ANGLE        START_KICK_ADV_ANGLE_Q15
#define START_OL_IQ                 START_OL_IQ_Q15
#define START_OL_STEP_INIT          START_OL_STEP_INIT_Q15
#define START_OL_STEP_MAX           START_OL_STEP_MAX_Q15
#define START_OL_EXIT_SPD           START_OL_EXIT_SPD_RPM
#define START_OL_MAX_CYCLES         START_OL_MAX_CYCLES_CFG
#define POS_HOLD_IQ_BRAKE           0
#define HOLD_TARGET_HALL            1
#define HOLD_HALL_STABLE_CNT        8
#define HOLD_TARGET_BIAS            900
#define MOTOR_CMD_ACTIVE_TH         40
#define STOP_PWM_OFF_SPD            20
#define STOP_BRAKE_ENTRY_SPD        220
#define STOP_BRAKE_EXIT_SPD         80
#define STOP_BRAKE_IQ               180
#define STOP_BRAKE_MAX_CYCLES       700
#define STOP_PARK_ENABLE            1
#define STOP_PARK_DELAY_CYCLES      180
#define STOP_PARK_IQ                70
#define STOP_PARK_ANGLE_DEAD        700
#define STOP_PARK_ERR_HYST          120

static uint8_t s_pos_hold_active = 0;
static uint16_t s_start_align_cnt = 0;
static int16_t s_pos_hold_target_angle = 0;
static uint8_t s_hold_hall_stable_cnt = 0;
static uint8_t s_stop_brake_active = 0;
static uint16_t s_stop_brake_cnt = 0;
static uint16_t s_stop_park_delay_cnt = 0;
static uint16_t s_stop_park_prev_abs_err = 0;
static int8_t s_stop_park_polarity = 1;
static uint8_t s_stop_park_flip_done = 0;
static uint8_t s_start_ol_active = 0;
static uint16_t s_start_ol_cnt = 0;
static int16_t s_start_ol_angle = 0;
static int16_t s_start_ol_step = START_OL_STEP_INIT;
static uint8_t s_prev_pos_hold_cmd = 0;
static uint16_t s_pos_coarse_settle_cnt = 0;
extern int16_t HallAngleOffset;

/*------------------ Private functions ----------------*/
void Motor_Drive(void);
void Motor_Model(u8 state);
void MotorIdle(void);
void MotorBrake(void);
void MotorRun(void);
void MotorError(void);

static int16_t WrapAngleErr(int16_t target, int16_t angle)
{
    int32_t err = (int32_t)target - (int32_t)angle;

    if (err > 32767)
    {
        err -= 65536;
    }
    else if (err < -32768)
    {
        err += 65536;
    }
    return (int16_t)err;
}

static int32_t Abs32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int16_t ClampToS16(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16_t)value;
}

static int32_t CurrentIIRStep(int32_t state, int32_t sample, uint8_t shift)
{
    if (shift == 0U)
    {
        return sample;
    }
    return state + ((sample - state) >> shift);
}

static uint8_t GetHallNextState(uint8_t hall)
{
    switch (hall)
    {
    case 1: return 3;
    case 3: return 2;
    case 2: return 6;
    case 6: return 4;
    case 4: return 5;
    case 5: return 1;
    default: return HOLD_TARGET_HALL;
    }
}

static uint8_t GetHallPrevState(uint8_t hall)
{
    switch (hall)
    {
    case 1: return 5;
    case 3: return 1;
    case 2: return 3;
    case 6: return 2;
    case 4: return 6;
    case 5: return 4;
    default: return HOLD_TARGET_HALL;
    }
}

static int16_t GetHallCenterAngleByDir(uint8_t hall, int8_t dir)
{
    int32_t center;

    if ((hall == 0U) || (hall == 7U))
    {
        hall = HALL_ReadHallPorts();
    }
    if ((hall == 0U) || (hall == 7U))
    {
        hall = HOLD_TARGET_HALL;
    }

    if (dir < 0)
    {
        center = (int32_t)HALL1.CCWAngleTab[hall] + 5460 + HallAngleOffset;
    }
    else
    {
        center = (int32_t)HALL1.CWAngleTab[hall] + 5460 + HallAngleOffset;
    }

    if (center > 32767)
    {
        center -= 65536;
    }
    else if (center < -32768)
    {
        center += 65536;
    }

    return (int16_t)center;
}

static int16_t GetFixedAlignAngle(void)
{
    uint8_t hall_now = HALL1.RunHallValue;
    uint8_t hall_target;
    int16_t center;

    if ((hall_now == 0U) || (hall_now == 7U))
    {
        hall_now = HALL_ReadHallPorts();
    }
    if ((hall_now == 0U) || (hall_now == 7U))
    {
        hall_now = HOLD_TARGET_HALL;
    }

    if (HALL1.CMDDIR < 0)
    {
        hall_target = GetHallPrevState(hall_now);
    }
    else
    {
        hall_target = GetHallNextState(hall_now);
    }

    if (HALL1.HallState != 0U)
    {
        if (HALL1.CMDDIR < 0)
        {
            center = (int16_t)(HALL1.CCWAngleTab[hall_target] + 5460 + HallAngleOffset);
        }
        else
        {
            center = (int16_t)(HALL1.CWAngleTab[hall_target] + 5460 + HallAngleOffset);
        }
    }
    else if (HALL1.CMDDIR < 0)
    {
        center = (int16_t)(HALL1.CCWAngleTab[hall_target] + 5460 + HallAngleOffset);
    }
    else
    {
        center = (int16_t)(HALL1.CWAngleTab[hall_target] + 5460 + HallAngleOffset);
    }

    if (HALL1.CMDDIR < 0)
    {
        return (int16_t)(center - HOLD_TARGET_BIAS);
    }
    return (int16_t)(center + HOLD_TARGET_BIAS);
}

static uint8_t MotorCmdActive(void)
{
#if POSITION_LOOP_ENABLE
    return (RP.OutEn != 0U) ? 1U : 0U;
#else
    return (RPValue.Act > MOTOR_CMD_ACTIVE_TH) ? 1U : 0U;
#endif
}

static uint8_t CurrentOffsetTrackEnabled(int32_t iu_raw_q3, int32_t iv_raw_q3)
{
#if CURRENT_OFFSET_TRACK_ENABLE
    if (MotorState != IDLESTATE)
    {
        return 0U;
    }
    if ((TIM1->BDTR & TIM_BDTR_MOE) != 0U)
    {
        return 0U;
    }
    if (MotorCmdActive() != 0U)
    {
        return 0U;
    }
    if (Abs32((int32_t)HALL1.SpeedTemp) > CURRENT_OFFSET_TRACK_MAX_RPM)
    {
        return 0U;
    }
    if ((Abs32(iu_raw_q3) > CURRENT_OFFSET_TRACK_TH_Q3) ||
        (Abs32(iv_raw_q3) > CURRENT_OFFSET_TRACK_TH_Q3))
    {
        return 0U;
    }
    return 1U;
#else
    (void)iu_raw_q3;
    (void)iv_raw_q3;
    return 0U;
#endif
}

static uint8_t CurrentRuntimeFilterEnabled(void)
{
#if CURRENT_RUNTIME_FILTER_ENABLE
    if (MotorState != RUNSTATE)
    {
        return 0U;
    }
    if (s_pos_hold_active != 0U)
    {
        return 0U;
    }
    if (s_start_ol_active != 0U)
    {
        return 0U;
    }
    return 1U;
#else
    return 0U;
#endif
}

/****************************************************************
    Function: Motor_Drive
****************************************************************/
void Motor_Drive(void)
{
    static uint16_t u16Cnt = 0;
    static uint8_t s_prev_hold = 0;
    static uint8_t s_pos_move_active = 0;
    static uint8_t s_current_filter_ready = 0;
    uint8_t hall_now;
    uint8_t cmd_active;
    int16_t ctrlAngle;
    int16_t hold_iq_cmd = 0;
    int16_t start_dir = (HALL1.CMDDIR >= 0) ? 1 : -1;
    int32_t iu_adc_q3;
    int32_t iv_adc_q3;
    int32_t iu_raw_q3;
    int32_t iv_raw_q3;

    TIMFlag.PWMIn = 1;

    Time5msCnt++;
    if (Time5msCnt >= 80)
    {
        Time5msCnt = 0;
        TIMFlag.Delay5ms = 1;
    }

    Time200msCnt++;
    if (Time200msCnt >= 3200)
    {
        Time200msCnt = 0;
        TIMFlag.Delay200ms = 1;
    }

    iu_adc_q3 = (int32_t)GET_ADC_INJECT_VALUE(ADC1, IR_U_INJECT_RANK) << 3;
    iv_adc_q3 = (int32_t)GET_ADC_INJECT_VALUE(ADC1, IR_V_INJECT_RANK) << 3;

    if (u16Cnt < CURRENT_OFFSET_CAL_SAMPLES)
    {
        u16Cnt++;
        u32IuSum += (uint32_t)(iu_adc_q3 >> 3);
        u32IvSum += (uint32_t)(iv_adc_q3 >> 3);
        ADC_Structure.IU = 0;
        ADC_Structure.IV = 0;
    }
    else if (u16Cnt == CURRENT_OFFSET_CAL_SAMPLES)
    {
        u16Cnt++;
        s32IuOffsetQ3 = (int32_t)(((u32IuSum << 3) + (CURRENT_OFFSET_CAL_SAMPLES >> 1)) / CURRENT_OFFSET_CAL_SAMPLES);
        s32IvOffsetQ3 = (int32_t)(((u32IvSum << 3) + (CURRENT_OFFSET_CAL_SAMPLES >> 1)) / CURRENT_OFFSET_CAL_SAMPLES);
        u32IuSum = 0;
        u32IvSum = 0;
        s32IuFiltQ3 = 0;
        s32IvFiltQ3 = 0;
        s_current_filter_ready = 0U;
        ADC_Structure.IU = 0;
        ADC_Structure.IV = 0;
    }
    else
    {
        iu_raw_q3 = s32IuOffsetQ3 - iu_adc_q3;
        iv_raw_q3 = s32IvOffsetQ3 - iv_adc_q3;

        if (CurrentOffsetTrackEnabled(iu_raw_q3, iv_raw_q3))
        {
            s32IuOffsetQ3 = CurrentIIRStep(s32IuOffsetQ3, iu_adc_q3, CURRENT_OFFSET_TRACK_SHIFT);
            s32IvOffsetQ3 = CurrentIIRStep(s32IvOffsetQ3, iv_adc_q3, CURRENT_OFFSET_TRACK_SHIFT);
            iu_raw_q3 = s32IuOffsetQ3 - iu_adc_q3;
            iv_raw_q3 = s32IvOffsetQ3 - iv_adc_q3;
        }

        if (CurrentRuntimeFilterEnabled() != 0U)
        {
            if (s_current_filter_ready == 0U)
            {
                s32IuFiltQ3 = iu_raw_q3;
                s32IvFiltQ3 = iv_raw_q3;
                s_current_filter_ready = 1U;
            }
            else
            {
                s32IuFiltQ3 = CurrentIIRStep(s32IuFiltQ3, iu_raw_q3, CURRENT_FILTER_SHIFT);
                s32IvFiltQ3 = CurrentIIRStep(s32IvFiltQ3, iv_raw_q3, CURRENT_FILTER_SHIFT);
            }

            ADC_Structure.IU = ClampToS16(s32IuFiltQ3);
            ADC_Structure.IV = ClampToS16(s32IvFiltQ3);
        }
        else
        {
            s32IuFiltQ3 = iu_raw_q3;
            s32IvFiltQ3 = iv_raw_q3;
            s_current_filter_ready = 0U;
            ADC_Structure.IU = ClampToS16(iu_raw_q3);
            ADC_Structure.IV = ClampToS16(iv_raw_q3);
        }
        ADC_Structure.VBusInput = GET_ADC_VALUE(VBUS_CHANNEL);
        ADC_Structure.SPEED = GET_ADC_VALUE(VR_CHANNEL);
    }

    Motor_Model(MotorState);
    cmd_active = MotorCmdActive();

#if POSITION_COARSE_MODE
    if ((PositionHoldEnable != 0U) && cmd_active)
    {
        if (s_prev_pos_hold_cmd == 0U)
        {
            s_pos_coarse_settle_cnt = POSITION_COARSE_SETTLE_CYCLES;
        }
        s_prev_pos_hold_cmd = 1U;
    }
    else
    {
        s_prev_pos_hold_cmd = 0U;
        s_pos_coarse_settle_cnt = 0U;
    }
#endif

    HALLModuleCalc(&HALL1);
    HALLCheck(&HALL1);
    hall_now = HALL_ReadHallPorts();
    if (hall_now == HOLD_TARGET_HALL)
    {
        if (s_hold_hall_stable_cnt < 250U) { s_hold_hall_stable_cnt++; }
    }
    else
    {
        s_hold_hall_stable_cnt = 0;
    }

    /* Select control angle and torque command in hold mode. */
    if ((PositionHoldEnable != 0U) && cmd_active)
    {
        s_pos_hold_active = 1;
        s_start_align_cnt = START_ALIGN_PWM_CYCLES;
        s_start_ol_active = 0;
        s_start_ol_cnt = 0;
        s_start_ol_step = START_OL_STEP_INIT;
        s_pos_hold_target_angle = PositionHoldTargetAngle;
    }
    ctrlAngle = HALL1.FocAngle;
    if (s_pos_hold_active)
    {
        if (!cmd_active)
        {
            if (s_stop_brake_active)
            {
                IqRef = -STOP_BRAKE_IQ;
                ctrlAngle = HALL1.Angle;
            }
#if STOP_PARK_ENABLE
            else if (s_stop_park_delay_cnt < STOP_PARK_DELAY_CYCLES)
            {
                s_stop_park_delay_cnt++;
                IqRef = 0;
                ctrlAngle = HALL1.Angle;
            }
            else
            {
                int16_t angle_err = WrapAngleErr(s_pos_hold_target_angle, HALL1.Angle);
                uint16_t abs_err = (angle_err >= 0) ? (uint16_t)angle_err : (uint16_t)(-angle_err);

                if ((s_stop_park_prev_abs_err != 0U) &&
                    (abs_err > (uint16_t)(s_stop_park_prev_abs_err + STOP_PARK_ERR_HYST)) &&
                    (s_stop_park_flip_done == 0U))
                {
                    s_stop_park_polarity = (int8_t)(-s_stop_park_polarity);
                    s_stop_park_flip_done = 1;
                }

                if (abs_err <= STOP_PARK_ANGLE_DEAD)
                {
                    IqRef = 0;
                }
                else
                {
                    int16_t sign = (angle_err >= 0) ? 1 : -1;
                    IqRef = (int16_t)(sign * s_stop_park_polarity * STOP_PARK_IQ);
                }
                s_stop_park_prev_abs_err = abs_err;
                ctrlAngle = s_pos_hold_target_angle;
            }
#else
            else
            {
                IqRef = 0;
                ctrlAngle = HALL1.Angle;
            }
#endif
        }
        else
        {
#if POSITION_COARSE_MODE
            int16_t angle_fdb = GetHallCenterAngleByDir(HALL1.RunHallValue, POSITION_HOLD_DIR);
            int16_t angle_err = WrapAngleErr(s_pos_hold_target_angle, angle_fdb);
            uint16_t abs_angle_err = (angle_err >= 0) ? (uint16_t)angle_err : (uint16_t)(-angle_err);
#else
            int16_t angle_err = WrapAngleErr(s_pos_hold_target_angle, HALL1.Angle);
            uint16_t abs_angle_err = (angle_err >= 0) ? (uint16_t)angle_err : (uint16_t)(-angle_err);
#endif

            if ((PositionHoldEnable != 0U) && cmd_active)
            {
#if POSITION_COARSE_MODE
                hold_iq_cmd = POSITION_COARSE_HOLD_IQ_Q15;
                s_pos_move_active = 0U;

                if (s_pos_coarse_settle_cnt > 0U)
                {
                    s_pos_coarse_settle_cnt--;
                    IqRef = (POSITION_HOLD_DIR >= 0) ? POSITION_COARSE_SETTLE_IQ_Q15 : -POSITION_COARSE_SETTLE_IQ_Q15;
                }
                else if (angle_err > POSITION_COARSE_HOLD_DEAD_Q15)
                {
                    IqRef = hold_iq_cmd;
                }
                else if (angle_err < -POSITION_COARSE_HOLD_DEAD_Q15)
                {
                    IqRef = -hold_iq_cmd;
                }
                else
                {
                    IqRef = 0;
                }
                ctrlAngle = s_pos_hold_target_angle;
#else
                if (s_pos_move_active == 0U)
                {
                    if (abs_angle_err >= POSITION_MOVE_ENTER_Q15)
                    {
                        s_pos_move_active = 1U;
                    }
                }
                else if (abs_angle_err <= POSITION_MOVE_EXIT_Q15)
                {
                    s_pos_move_active = 0U;
                }

                if (s_pos_move_active != 0U)
                {
                    int16_t dir = (angle_err >= 0) ? 1 : -1;
                    IqRef = (int16_t)(dir * POSITION_MOVE_IQ_Q15);
                    ctrlAngle = (int16_t)(HALL1.Angle + dir * POSITION_MOVE_ADV_Q15);
                }
                else
                {
                    hold_iq_cmd = PositionHoldIq;

                    if (angle_err > POSITION_ERR_DEAD_Q15)
                    {
                        IqRef = hold_iq_cmd;
                    }
                    else if (angle_err < -POSITION_ERR_DEAD_Q15)
                    {
                        IqRef = -hold_iq_cmd;
                    }
                    else
                    {
                        IqRef = 0;
                    }
                    ctrlAngle = s_pos_hold_target_angle;
                }
#endif
            }
            else if (s_start_align_cnt < START_ALIGN_PWM_CYCLES && cmd_active)
            {
                if (s_start_align_cnt < START_LOCK_PWM_CYCLES)
                {
                    hold_iq_cmd = POS_HOLD_IQ_START;
                    /* Lock first: move rotor close to target electrical angle. */
                    IqRef = (angle_err >= 0) ? hold_iq_cmd : -hold_iq_cmd;
                    ctrlAngle = s_pos_hold_target_angle;
                }
                else
                {
                    int16_t dir = (HALL1.CMDDIR >= 0) ? 1 : -1;
                    /* Short kick: guarantee standstill breakaway torque. */
                    IqRef = (int16_t)(dir * START_KICK_IQ);
                    ctrlAngle = (int16_t)(s_pos_hold_target_angle + dir * START_KICK_ADV_ANGLE);
                }
            }
            else
            {
                hold_iq_cmd = POS_HOLD_IQ_STOP;

                if (angle_err > POS_HOLD_ANGLE_DEAD)
                {
                    IqRef = hold_iq_cmd;
                }
                else if (angle_err < -POS_HOLD_ANGLE_DEAD)
                {
                    IqRef = -hold_iq_cmd;
                }
                else
                {
                    IqRef = 0;
                }
                ctrlAngle = s_pos_hold_target_angle;
            }
        }
    }
    else if (s_start_ol_active && cmd_active)
    {
        s_start_ol_cnt++;
        if ((s_start_ol_cnt % START_OL_STEP_RAMP_CYCLES) == 0U)
        {
            if (s_start_ol_step < START_OL_STEP_MAX)
            {
                s_start_ol_step++;
            }
        }

        s_start_ol_angle = (int16_t)(s_start_ol_angle + start_dir * s_start_ol_step);
        ctrlAngle = s_start_ol_angle;
        IqRef = (int16_t)(start_dir * START_OL_IQ);

        if (((HALL1.NoEdgeCnt <= 4U) && (HALL1.SpeedTemp >= START_OL_EXIT_SPD)) ||
            (s_start_ol_cnt >= START_OL_MAX_CYCLES))
        {
            s_start_ol_active = 0;
            s_start_ol_cnt = 0;
            s_start_ol_step = START_OL_STEP_INIT;
            s_start_ol_angle = HALL1.Angle;
            CurIQ.qdSum = 0;
            CurIQ.qOut = 0;
            CurID.qdSum = 0;
            CurID.qOut = 0;
        }
    }

    Vector_Components = MCM_Trig_Functions(ctrlAngle);

    clarke1.As = ADC_Structure.IU;
    clarke1.Bs = ADC_Structure.IV;
    CLARKE_MACRO1(&clarke1);

    park1.Alpha = clarke1.Alpha;
    park1.Beta = clarke1.Beta;
    park1.Sin = Vector_Components.hSin;
    park1.Cos = Vector_Components.hCos;
    PARK_MACRO1(&park1);

    if (FAULT.Byte == 0)
    {
        if (s_pos_hold_active)
        {
            /* Avoid speed loop windup during position hold. */
            Speed.qdSum = 0;
            Speed.qOut = 0;
            CurID.qInRef = 0;
            CurIQ.qInRef = IqRef;
        }
        else
        {
            CurID.qInRef = IdRef;
            CurIQ.qInRef = Speed.qOut;
        }

        CurID.qInMeas = park1.Ds;
        CalcPI(&CurID);

        CurIQ.qInMeas = park1.Qs;
        CalcPI(&CurIQ);

        ipark1.Ds = CurID.qOut;
        ipark1.Qs = CurIQ.qOut;
        ipark1.Sin = Vector_Components.hSin;
        ipark1.Cos = Vector_Components.hCos;
        IPARK_MACRO1(&ipark1);

        pwm_gen.Alpha = ipark1.Alpha;
        pwm_gen.Beta = ipark1.Beta;
        PWM_GEN_calc(&pwm_gen);

        Update_PWM(&pwm_gen);
    }
    else
    {
        MotorState = ERRORSTATE;
    }

    if (s_prev_hold != s_pos_hold_active)
    {
        CurIQ.qdSum = 0;
        CurIQ.qOut = 0;
        CurID.qdSum = 0;
        CurID.qOut = 0;
        s_pos_move_active = 0U;
        s_prev_hold = s_pos_hold_active;
    }
}

/****************************************************************
    Function: Motor_Model
****************************************************************/
void Motor_Model(u8 state)
{
    switch (state)
    {
    case IDLESTATE:
        MotorIdle();
        break;
    case BRAKESTATE:
        MotorBrake();
        break;
    case RUNSTATE:
        MotorRun();
        break;
    case ERRORSTATE:
        MotorError();
        break;
    default:
        MotorState = IDLESTATE;
        break;
    }
}

/****************************************************************
    Function: MotorIdle
****************************************************************/
void MotorIdle(void)
{
    if (MotorCmdActive())
    {
        s_stop_brake_active = 0;
        s_stop_brake_cnt = 0;
        s_stop_park_delay_cnt = 0;
        s_stop_park_prev_abs_err = 0;
        s_stop_park_polarity = 1;
        s_stop_park_flip_done = 0;

        /* Start alignment: always align to the same angle before run. */
        if (s_start_align_cnt < START_ALIGN_PWM_CYCLES)
        {
            if (s_start_align_cnt == 0U)
            {
                s_pos_hold_target_angle = GetFixedAlignAngle();
                s_start_ol_active = 0;
                s_start_ol_cnt = 0;
                s_start_ol_step = START_OL_STEP_INIT;
                s_start_ol_angle = s_pos_hold_target_angle;
            }
            s_pos_hold_active = 1;
            s_start_align_cnt++;
            TIM_CtrlPWMOutputs(TIM1, ENABLE);
            MotorState = IDLESTATE;
        }
        else
        {
            s_pos_hold_active = 0;
            s_start_ol_active = 1;
            s_start_ol_cnt = 0;
            s_start_ol_step = START_OL_STEP_INIT;
            s_start_ol_angle = s_pos_hold_target_angle;
            TIM_CtrlPWMOutputs(TIM1, ENABLE);
            MotorState = RUNSTATE;
        }
    }
    else
    {
        s_pos_hold_target_angle = GetFixedAlignAngle();
        s_start_align_cnt = 0;
        s_hold_hall_stable_cnt = 0;
        s_start_ol_active = 0;
        s_start_ol_cnt = 0;
        s_start_ol_step = START_OL_STEP_INIT;
        if (s_stop_brake_active == 0U)
        {
            s_stop_park_delay_cnt = 0;
            s_stop_park_prev_abs_err = 0;
            s_stop_park_polarity = 1;
            s_stop_park_flip_done = 0;
        }
        if ((HALL1.SpeedTemp >= STOP_BRAKE_ENTRY_SPD) && (s_stop_brake_cnt < STOP_BRAKE_MAX_CYCLES))
        {
            s_stop_brake_active = 1;
            s_stop_brake_cnt++;
            s_pos_hold_active = 1;
            TIM_CtrlPWMOutputs(TIM1, ENABLE);
        }
        else if ((HALL1.SpeedTemp > STOP_BRAKE_EXIT_SPD) &&
                 (s_stop_brake_active != 0U) &&
                 (s_stop_brake_cnt < STOP_BRAKE_MAX_CYCLES))
        {
            s_stop_brake_cnt++;
            s_pos_hold_active = 1;
            TIM_CtrlPWMOutputs(TIM1, ENABLE);
        }
        else
        {
            s_stop_brake_active = 0;
            s_stop_brake_cnt = 0;
#if STOP_PARK_ENABLE
            s_pos_hold_active = 1;
            TIM_CtrlPWMOutputs(TIM1, ENABLE);
#else
            s_pos_hold_active = 0;
            TIM_CtrlPWMOutputs(TIM1, DISABLE);
            CurIQ.qdSum = 0;
            CurIQ.qOut = 0;
            CurID.qdSum = 0;
            CurID.qOut = 0;
            Speed.qdSum = 0;
            Speed.qOut = 0;
#endif
        }
        MotorState = IDLESTATE;
    }
}

/****************************************************************
    Function: MotorBrake
****************************************************************/
void MotorBrake(void)
{
    if (HALL1.SpeedTemp >= 100)
    {
        TIM_CtrlPWMOutputs(TIM1, DISABLE);
        MotorState = BRAKESTATE;
    }
    else
    {
        TIM_CtrlPWMOutputs(TIM1, ENABLE);
        HALL1.CMDDIR = -HALL1.CMDDIR;
        MotorState = RUNSTATE;

        CurIQ.qdSum = 0;
        CurIQ.qOut = 0;
        CurID.qdSum = 0;
        CurID.qOut = 0;
        Speed.qdSum = 0;
        Speed.qOut = 0;
    }
}

/****************************************************************
    Function: MotorRun
****************************************************************/
void MotorRun(void)
{
    if (MotorCmdActive())
    {
        s_pos_hold_active = 0;
        MotorState = RUNSTATE;
    }
    else
    {
        s_start_align_cnt = 0;
        s_stop_brake_active = 0;
        s_stop_brake_cnt = 0;
        s_start_ol_active = 0;
        s_start_ol_cnt = 0;
        s_start_ol_step = START_OL_STEP_INIT;
        s_stop_park_delay_cnt = 0;
        s_stop_park_prev_abs_err = 0;
        s_stop_park_polarity = 1;
        s_stop_park_flip_done = 0;
        s_pos_hold_active = 0;
        TIM_CtrlPWMOutputs(TIM1, DISABLE);
        CurIQ.qdSum = 0;
        CurIQ.qOut = 0;
        CurID.qdSum = 0;
        CurID.qOut = 0;
        Speed.qdSum = 0;
        Speed.qOut = 0;
        MotorState = IDLESTATE;
    }
}

/****************************************************************
    Function: MotorError
****************************************************************/
void MotorError(void)
{
    if (MotorCmdActive())
    {
        TIM_CtrlPWMOutputs(TIM1, DISABLE);
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        TIM1->CCR1FALL = 0;
        TIM1->CCR2FALL = 0;
        TIM1->CCR3FALL = 0;

        CurIQ.qdSum = 0;
        CurIQ.qOut = 0;
        CurID.qdSum = 0;
        CurID.qOut = 0;
        Speed.qdSum = 0;
        Speed.qOut = 0;

        s_pos_hold_active = 0;
        s_start_align_cnt = 0;
        MotorState = ERRORSTATE;
    }
    else
    {
        FAULT.Byte = 0;
        s_pos_hold_active = 0;
        s_start_align_cnt = 0;
        s_stop_brake_active = 0;
        s_stop_brake_cnt = 0;
        s_stop_park_delay_cnt = 0;
        s_stop_park_prev_abs_err = 0;
        s_stop_park_polarity = 1;
        s_stop_park_flip_done = 0;
        MotorState = IDLESTATE;
    }
}
