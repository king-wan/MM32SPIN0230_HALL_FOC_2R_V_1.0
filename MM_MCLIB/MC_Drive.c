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
uint16_t u16IuOffset = 0;
uint16_t u16IvOffset = 0;

/* Fixed-angle start/stop position hold */
#define POS_HOLD_ENTRY_SPD          120
#define POS_HOLD_IQ_STOP            220
#define POS_HOLD_IQ_START           320
#define POS_HOLD_ANGLE_DEAD         900
#define START_ALIGN_PWM_CYCLES      1200
#define POS_HOLD_IQ_BRAKE           260
#define HOLD_TARGET_HALL            1
#define HOLD_HALL_STABLE_CNT        8
#define HOLD_TARGET_BIAS            900

static uint8_t s_pos_hold_active = 0;
static uint16_t s_start_align_cnt = 0;
static int16_t s_pos_hold_target_angle = 0;
static uint8_t s_hold_hall_stable_cnt = 0;
static uint8_t s_hold_hall_reached = 0;

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

static int16_t GetFixedAlignAngle(void)
{
    if (HALL1.CMDDIR < 0)
    {
        return (int16_t)(HALL1.CCWAngleTab[HOLD_TARGET_HALL] + 5460 - HOLD_TARGET_BIAS);
    }
    return (int16_t)(HALL1.CWAngleTab[HOLD_TARGET_HALL] + 5460 + HOLD_TARGET_BIAS);
}

/****************************************************************
    Function: Motor_Drive
****************************************************************/
void Motor_Drive(void)
{
    static uint16_t u16Cnt = 0;
    static uint8_t s_prev_hold = 0;
    uint8_t hall_now;
    int16_t ctrlAngle;
    int16_t hold_iq_cmd = 0;

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

    if (u16Cnt <= 127)
    {
        u16Cnt++;
        u32IuSum += (int16_t)GET_ADC_VALUE(IR_U_CHANNEL);
        u32IvSum += (int16_t)GET_ADC_VALUE(IR_V_CHANNEL);
    }
    else if (u16Cnt == 128)
    {
        u16Cnt++;
        u16IuOffset = u32IuSum >> 4;
        u16IvOffset = u32IvSum >> 4;
        u32IuSum = 0;
        u32IvSum = 0;
    }
    else
    {
        ADC_Structure.IU = u16IuOffset - (GET_ADC_VALUE(IR_U_CHANNEL) << 3);
        ADC_Structure.IV = u16IvOffset - (GET_ADC_VALUE(IR_V_CHANNEL) << 3);
        ADC_Structure.VBusInput = GET_ADC_VALUE(VBUS_CHANNEL);
        ADC_Structure.SPEED = GET_ADC_VALUE(VR_CHANNEL);
    }

    Motor_Model(MotorState);

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
        s_hold_hall_reached = 0;
    }
    if (s_hold_hall_stable_cnt >= HOLD_HALL_STABLE_CNT)
    {
        s_hold_hall_reached = 1;
    }

    /* Select control angle and torque command in hold mode. */
    ctrlAngle = HALL1.Angle;
    if (s_pos_hold_active)
    {
        /* Stop phase 1: active electrical braking before angle lock. */
        if ((!RP.OutEn) && (HALL1.SpeedTemp > POS_HOLD_ENTRY_SPD))
        {
            IqRef = -POS_HOLD_IQ_BRAKE;
            ctrlAngle = HALL1.Angle;
        }
        else
        {
            int16_t angle_err = WrapAngleErr(s_pos_hold_target_angle, HALL1.Angle);

            if ((s_start_align_cnt < START_ALIGN_PWM_CYCLES && RP.OutEn) || (!s_hold_hall_reached))
            {
                hold_iq_cmd = POS_HOLD_IQ_START;
            }
            else
            {
                hold_iq_cmd = POS_HOLD_IQ_STOP;
            }

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
    s_pos_hold_target_angle = GetFixedAlignAngle();

    if (RP.OutEn)
    {
        /* Start alignment: always align to the same angle before run. */
        if ((s_start_align_cnt < START_ALIGN_PWM_CYCLES) || (!s_hold_hall_reached))
        {
            s_pos_hold_active = 1;
            s_start_align_cnt++;
            TIM_CtrlPWMOutputs(TIM1, ENABLE);
            MotorState = IDLESTATE;
        }
        else
        {
            s_pos_hold_active = 0;
            TIM_CtrlPWMOutputs(TIM1, ENABLE);
            MotorState = RUNSTATE;
        }
    }
    else
    {
        s_start_align_cnt = 0;
        s_hold_hall_reached = 0;
        s_hold_hall_stable_cnt = 0;
        /* Stop request: always keep position-hold state enabled.
           Motor_Drive will brake first, then lock to fixed angle. */
        s_pos_hold_active = 1;
        TIM_CtrlPWMOutputs(TIM1, ENABLE);
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
    if (RP.OutEn)
    {
        s_pos_hold_active = 0;
        MotorState = RUNSTATE;
    }
    else
    {
        s_start_align_cnt = 0;
        MotorState = IDLESTATE;
    }
}

/****************************************************************
    Function: MotorError
****************************************************************/
void MotorError(void)
{
    if (RP.OutEn)
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
        MotorState = IDLESTATE;
    }
}
