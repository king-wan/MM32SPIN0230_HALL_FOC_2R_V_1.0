/*-------------------- Includes -----------------------*/
#include "drv_inc.h"
#include "HallHandle.h"
#include "MC_Drive.h"
#include "user_function.h"
#include "parameter.h"
#include "hall_tune.h"
#include "FOC_Math.h"
#include "PID.h"
#include "Diagnose.h"

/*------------------- Private variables ---------------*/
HALLType HALL1;
SectorType SectorStudy;
int16_t CWShift = 0;
int16_t CCWShift = -1200;
int16_t HallAngleOffset = HALL_EANGLE_OFFSET_Q15;
uint8_t ReadHallValue;

volatile uint8_t g_hall_edge_head = 0;
volatile uint8_t g_hall_edge_tail = 0;
volatile uint8_t g_hall_edge_count = 0;
volatile uint8_t g_hall_edge_old_buf[32] = {0};
volatile uint8_t g_hall_edge_new_buf[32] = {0};

#define HALL_Q15_15DEG               2730
#define HALL_Q15_30DEG               5460
#define HALL_Q15_60DEG               10922
#define HALL_Q15_FULL_TURN           65536L
#define HALL_SECTOR_NUM              6U
#define HALL_TIMEOUT_PWM_CYCLES      2500U
#define HALL_MIN_VALID_PERIOD_US     20U
#define HALL_HALF_SECTOR_SPD_TH      500
#define HALL_FULL_SECTOR_SPD_TH      800
#define HALL_PERIOD_STALE_FACTOR     2U

/*------------------ Private functions ----------------*/
void HALLModuleInit(HALLType *u);
void HALLModuleCalc(HALLType *u);
uint8_t HALL_ReadHallPorts(void);
void HALLCheck(HALLType *u);

static int16_t Hall_WrapAngle(int32_t angle)
{
    while (angle > 32767)
    {
        angle -= 65536;
    }
    while (angle < -32768)
    {
        angle += 65536;
    }
    return (int16_t)angle;
}

static uint8_t Hall_IsValidValue(uint8_t hall)
{
    return (uint8_t)((hall != 0U) && (hall != 7U));
}

static uint8_t Hall_GetNextState(uint8_t hall)
{
    switch (hall)
    {
    case 1: return 3;
    case 3: return 2;
    case 2: return 6;
    case 6: return 4;
    case 4: return 5;
    case 5: return 1;
    default: return 0;
    }
}

static uint8_t Hall_GetPrevState(uint8_t hall)
{
    switch (hall)
    {
    case 1: return 5;
    case 3: return 1;
    case 2: return 3;
    case 6: return 2;
    case 4: return 6;
    case 5: return 4;
    default: return 0;
    }
}

static int8_t Hall_GetEdgeDirection(uint8_t old_hall, uint8_t new_hall)
{
    if (new_hall == Hall_GetNextState(old_hall))
    {
        return 1;
    }
    if (new_hall == Hall_GetPrevState(old_hall))
    {
        return -1;
    }
    return 0;
}

static int16_t Hall_GetCenterAngle(const HALLType *u, uint8_t hall)
{
    int32_t angle;

    if (u->CMDDIR < 0)
    {
        angle = (int32_t)u->CCWAngleTab[hall] + (int32_t)HALL_Q15_30DEG + (int32_t)CCWShift + (int32_t)HallAngleOffset;
    }
    else
    {
        angle = (int32_t)u->CWAngleTab[hall] + (int32_t)HALL_Q15_30DEG + (int32_t)CWShift + (int32_t)HallAngleOffset;
    }

    return Hall_WrapAngle(angle);
}

static void Hall_UpdateAngleWindow(HALLType *u, int16_t center, uint8_t use_half_sector)
{
    int16_t half_width = use_half_sector ? HALL_Q15_15DEG : HALL_Q15_30DEG;

    u->AngleCenter = center;
    u->AngleLowLimit = Hall_WrapAngle((int32_t)center - half_width);
    u->AngleHighLimit = Hall_WrapAngle((int32_t)center + half_width);
    u->UseHalfSector = use_half_sector;
}

static uint8_t Hall_IsAngleWithinWindow(const HALLType *u, int16_t angle)
{
    if (u->AngleLowLimit <= u->AngleHighLimit)
    {
        return (uint8_t)((angle >= u->AngleLowLimit) && (angle <= u->AngleHighLimit));
    }
    return (uint8_t)((angle >= u->AngleLowLimit) || (angle <= u->AngleHighLimit));
}

static int16_t Hall_ClampAngleToWindow(const HALLType *u, int16_t angle, int32_t dpp)
{
    if (Hall_IsAngleWithinWindow(u, angle) != 0U)
    {
        return angle;
    }

    if (dpp >= 0)
    {
        return u->AngleHighLimit;
    }
    return u->AngleLowLimit;
}

static int16_t Hall_CalcSpeedAdvance(const HALLType *u)
{
#if HALL_SPEED_ADV_ENABLE
    int32_t speed_mag = u->SpeedFilt;
    int32_t advance;

    if (speed_mag < HALL_SPEED_ADV_MIN_RPM10)
    {
        return 0;
    }

    advance = ((speed_mag * (int32_t)POLEPAIRS * HALL_Q15_FULL_TURN * (int32_t)HALL_SPEED_ADV_DELAY_US) + 3000000L) / 6000000L;
    if (advance > HALL_SPEED_ADV_MAX_Q15)
    {
        advance = HALL_SPEED_ADV_MAX_Q15;
    }

    if (u->CMDDIR < 0)
    {
        advance = -advance;
    }

    return (int16_t)advance;
#else
    (void)u;
    return 0;
#endif
}

static void Hall_UpdateFocAngle(HALLType *u)
{
    u->AngleAdvance = Hall_CalcSpeedAdvance(u);
    u->FocAngle = Hall_WrapAngle((int32_t)u->Angle + (int32_t)u->AngleAdvance);
}

static void Hall_UpdateSpeedFromPeriod(HALLType *u)
{
    uint8_t idx;
    uint32_t hall_sum = 0;

    for (idx = 0; idx < HALL_SECTOR_NUM; idx++)
    {
        hall_sum += u->HallTime[idx];
    }

    if (hall_sum == 0U)
    {
        hall_sum = 1U;
    }

    u->HallTimeSum = hall_sum;
    u->SpeedTemp = (int16_t)Division(SpeedGain, (int32_t)hall_sum);
    if (u->SpeedFilt == 0)
    {
        u->SpeedFilt = u->SpeedTemp;
    }
    else
    {
        u->SpeedFilt = (int16_t)(((int32_t)u->SpeedFilt * 3 + (int32_t)u->SpeedTemp) >> 2);
    }
    u->PredictedDpp = (int16_t)Division(4096000, (int32_t)hall_sum);
    u->IncAngle = u->PredictedDpp;
    u->IncAngleMax = HALL_Q15_60DEG;

    if (u->PredictedDpp < 0)
    {
        u->PredictedDpp = (int16_t)(-u->PredictedDpp);
        u->IncAngle = (int16_t)(-u->PredictedDpp);
    }

    if (u->PredictedDpp == 0)
    {
        u->EdgeCycleTarget = HALL_TIMEOUT_PWM_CYCLES;
    }
    else
    {
        uint32_t sector_cycles = ((hall_sum / HALL_SECTOR_NUM) * PWMFREQ + 500000U) / 1000000U;
        if (sector_cycles == 0U)
        {
            sector_cycles = 1U;
        }
        u->EdgeCycleTarget = (uint16_t)sector_cycles;
    }
}

static void Hall_HandleTransition(HALLType *u, uint8_t new_hall)
{
    uint32_t period_us;
    int8_t edge_dir;
    uint8_t use_half_sector;
    int16_t synced_angle;
    uint8_t next_slot;
    uint32_t stale_limit;

    if (g_hall_edge_count >= 32U)
    {
        g_hall_edge_tail = (uint8_t)((g_hall_edge_tail + 1U) & 0x1FU);
        g_hall_edge_count = 31U;
    }
    g_hall_edge_old_buf[g_hall_edge_head] = u->PreHallValue;
    g_hall_edge_new_buf[g_hall_edge_head] = new_hall;
    g_hall_edge_head = (uint8_t)((g_hall_edge_head + 1U) & 0x1FU);
    g_hall_edge_count++;

    edge_dir = Hall_GetEdgeDirection(u->PreHallValue, new_hall);
    u->EdgeDir = edge_dir;
    u->RunHallValue = new_hall;
    u->NoEdgeCnt = 0;
    u->EdgeCycleCnt = 0;

    period_us = TIM2->CCR1;
    if (period_us < HALL_MIN_VALID_PERIOD_US)
    {
        period_us = HALL_MIN_VALID_PERIOD_US;
    }

    stale_limit = (u->HallTimeSum == 0U) ? (uint32_t)HALL_TIMEOUT_PWM_CYCLES : (u->HallTimeSum / HALL_SECTOR_NUM) * HALL_PERIOD_STALE_FACTOR;
    if ((u->HallTimeSum != 0U) && (period_us > stale_limit) && (stale_limit > 0U))
    {
        period_us = stale_limit;
    }

    next_slot = (uint8_t)((u->PeriodIndex + 1U) % HALL_SECTOR_NUM);
    u->PeriodIndex = next_slot;
    u->HallTime[next_slot] = period_us;

    Hall_UpdateSpeedFromPeriod(u);

    use_half_sector = (uint8_t)((u->SpeedTemp < HALL_HALF_SECTOR_SPD_TH) || (edge_dir == 0));
    if (u->SpeedTemp >= HALL_FULL_SECTOR_SPD_TH)
    {
        use_half_sector = 0U;
    }

    synced_angle = Hall_GetCenterAngle(u, new_hall);
    Hall_UpdateAngleWindow(u, synced_angle, use_half_sector);
    u->Angle = synced_angle;
    Hall_UpdateFocAngle(u);
    u->HallState = (edge_dir == 0) ? 0U : 1U;
}

/****************************************************************
    Function: HALLModuleInit
****************************************************************/
void HALLModuleInit(HALLType *u)
{
    uint8_t i;
    uint8_t hall_now;

    u->CMDDIR = MOTOR_DIR;
    u->CWAngleTab[1] = -21844;
    u->CWAngleTab[3] = -10922;
    u->CWAngleTab[2] = 0;
    u->CWAngleTab[6] = 10922;
    u->CWAngleTab[4] = 21844;
    u->CWAngleTab[5] = 32767;

    u->CCWAngleTab[5] = -21844;
    u->CCWAngleTab[4] = -10922;
    u->CCWAngleTab[6] = 0;
    u->CCWAngleTab[2] = 10922;
    u->CCWAngleTab[3] = 21844;
    u->CCWAngleTab[1] = 32767;

    hall_now = HALL_ReadHallPorts();
    if (Hall_IsValidValue(hall_now) == 0U)
    {
        hall_now = 1U;
        u->HallState = 0U;
    }
    else
    {
        u->HallState = 1U;
    }

    u->RunHallValue = hall_now;
    u->PreHallValue = hall_now;
    u->EdgeDir = 0;
    u->SpeedTemp = 0;
    u->SpeedFilt = 0;
    u->PredictedDpp = 0;
    u->IncAngle = 0;
    u->IncAngleMax = HALL_Q15_60DEG;
    u->Time100msCNT = 0;
    u->PeriodIndex = 0;
    u->NoEdgeCnt = 0;
    u->EdgeCycleCnt = 0;
    u->EdgeCycleTarget = HALL_TIMEOUT_PWM_CYCLES;
    u->HallTimeSum = 60000U;

    for (i = 0; i < 8U; i++)
    {
        u->HallTime[i] = 10000U;
    }

    u->Angle = Hall_GetCenterAngle(u, hall_now);
    Hall_UpdateAngleWindow(u, u->Angle, 1U);
    Hall_UpdateFocAngle(u);

    g_hall_edge_head = 0;
    g_hall_edge_tail = 0;
    g_hall_edge_count = 0;
}

/****************************************************************
    Function: HALLModuleCalc
****************************************************************/
void HALLModuleCalc(HALLType *u)
{
    int32_t predicted_dpp;
    uint8_t hall_now;

    hall_now = HALL_ReadHallPorts();
    ReadHallValue = hall_now;
    u->RunHallValue = hall_now;

    if ((Hall_IsValidValue(hall_now) != 0U) && (u->PreHallValue != hall_now))
    {
        Hall_HandleTransition(u, hall_now);
        u->PreHallValue = hall_now;
        return;
    }

    if (u->NoEdgeCnt < 0xFFFFU)
    {
        u->NoEdgeCnt++;
    }
    if (u->EdgeCycleCnt < 0xFFFFU)
    {
        u->EdgeCycleCnt++;
    }

    if (u->NoEdgeCnt >= HALL_TIMEOUT_PWM_CYCLES)
    {
        u->SpeedTemp = 0;
        u->SpeedFilt = 0;
        u->PredictedDpp = 0;
        u->IncAngle = 0;
        u->IncAngleMax = 0;
        u->HallState = 0U;
        u->Angle = Hall_GetCenterAngle(u, u->PreHallValue);
        Hall_UpdateAngleWindow(u, u->Angle, 1U);
        Hall_UpdateFocAngle(u);
        return;
    }

    predicted_dpp = u->PredictedDpp;
    if ((u->EdgeCycleTarget != 0U) && (u->EdgeCycleCnt > u->EdgeCycleTarget))
    {
        predicted_dpp = Division(HALL_Q15_60DEG, (int32_t)u->EdgeCycleCnt);
    }

    if (u->CMDDIR < 0)
    {
        predicted_dpp = -predicted_dpp;
    }

    u->IncAngle = (int16_t)predicted_dpp;
    u->Angle = Hall_ClampAngleToWindow(u, Hall_WrapAngle((int32_t)u->Angle + predicted_dpp), predicted_dpp);
    Hall_UpdateFocAngle(u);
}

/****************************************************************
    Function: HALL_ReadHallPorts
****************************************************************/
uint8_t HALL_ReadHallPorts(void)
{
    uint8_t HallA;
    uint8_t HallB;
    uint8_t HallC;
    uint8_t HallValue;

    HallA = GPIO_ReadInputDataBit(HALL_U_PORT, HALL_U_PIN);
    HallB = GPIO_ReadInputDataBit(HALL_V_PORT, HALL_V_PIN);
    HallC = GPIO_ReadInputDataBit(HALL_W_PORT, HALL_W_PIN);
    HallValue = (uint8_t)(HallA * 1U + HallB * 2U + HallC * 4U);

    return HallValue;
}

/****************************************************************
    Function: HALLCheck
****************************************************************/
void HALLCheck(HALLType *u)
{
    if (Hall_IsValidValue(u->RunHallValue) == 0U)
    {
        FAULT.bit.HallFlag = 1;
    }
}
