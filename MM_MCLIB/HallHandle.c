/*-------------------- Includes -----------------------*/
#include "drv_inc.h"
#include "HallHandle.h"
#include "MC_Drive.h"
#include "user_function.h"
#include "FOC_Math.h"
#include "PID.h"
#include "Diagnose.h"

/*------------------- Private variables ---------------*/
HALLType HALL1;
SectorType SectorStudy;
int16_t CWShift = 0;
int16_t CCWShift = -1200;
uint8_t ReadHallValue;

volatile uint8_t g_hall_edge_head = 0;
volatile uint8_t g_hall_edge_tail = 0;
volatile uint8_t g_hall_edge_count = 0;
volatile uint8_t g_hall_edge_old_buf[32] = {0};
volatile uint8_t g_hall_edge_new_buf[32] = {0};

#define HALL_NO_EDGE_TIMEOUT_CNT  2500U

/*------------------ Private functions ----------------*/
void HALLModuleInit(HALLType *u);
void HALLModuleCalc(HALLType *u);
uint8_t HALL_ReadHallPorts(void);
void HALLCheck(HALLType *u);

/****************************************************************
    Function: HALLModuleInit
****************************************************************/
void HALLModuleInit(HALLType *u)
{
    uint8_t i;

    u->RunHallValue = HALL_ReadHallPorts();
    u->PreHallValue = u->RunHallValue;
    u->CMDDIR = MOTOR_DIR;
    u->IncAngle = 5;
    u->IncAngleMax = 10922;
    u->SpeedTemp = 0;
    u->Time100msCNT = 0;
    u->HallTimeSum = 60000;

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

    if (u->CMDDIR == -1)
    {
        u->Angle = u->CCWAngleTab[HALL1.RunHallValue] + 5460;
    }
    else
    {
        u->Angle = u->CWAngleTab[HALL1.RunHallValue] + 5460;
    }

    for (i = 0; i < 8; i++)
    {
        u->HallTime[i] = 10000;
    }

    g_hall_edge_head = 0;
    g_hall_edge_tail = 0;
    g_hall_edge_count = 0;
}

/****************************************************************
    Function: HALLModuleCalc
****************************************************************/
void HALLModuleCalc(HALLType *u)
{
    static uint8_t i = 0;
    static uint16_t s_no_edge_cnt = 0;
    uint8_t j = 0;

    u->RunHallValue = HALL_ReadHallPorts();

    if (u->PreHallValue != u->RunHallValue)
    {
        if (g_hall_edge_count >= 32)
        {
            g_hall_edge_tail = (uint8_t)((g_hall_edge_tail + 1) & 0x1F);
            g_hall_edge_count = 31;
        }
        g_hall_edge_old_buf[g_hall_edge_head] = u->PreHallValue;
        g_hall_edge_new_buf[g_hall_edge_head] = u->RunHallValue;
        g_hall_edge_head = (uint8_t)((g_hall_edge_head + 1) & 0x1F);
        g_hall_edge_count++;

        s_no_edge_cnt = 0;
        u->Time100msCNT = 0;

        i++;
        if (i >= 6)
        {
            i = 0;
        }

        u->HallTime[i] = TIM2->CCR1;

        u->HallTimeSum = 0;
        for (j = 0; j < 6; j++)
        {
            u->HallTimeSum += u->HallTime[j];
        }

        if (u->HallTimeSum == 0)
        {
            u->HallTimeSum = 1;
        }

        u->SpeedTemp = Division(SpeedGain, u->HallTimeSum);

        u->IncAngle = Division(4096000, u->HallTimeSum);
        u->IncAngleMax = 10922;

        if (u->CMDDIR == -1)
        {
            u->Angle = u->CCWAngleTab[u->RunHallValue] + CCWShift;
        }
        else
        {
            u->Angle = u->CWAngleTab[u->RunHallValue] + CWShift;
        }
    }
    else
    {
        if (s_no_edge_cnt < 0xFFFFU) { s_no_edge_cnt++; }
        if (s_no_edge_cnt >= HALL_NO_EDGE_TIMEOUT_CNT)
        {
            /* No Hall edge for a while: treat as zero speed to avoid stale high-speed feedback. */
            u->SpeedTemp = 0;
            u->IncAngle = 0;
            u->IncAngleMax = 0;
        }
    }

    u->PreHallValue = u->RunHallValue;

    if ((u->IncAngleMax - u->IncAngle) >= 0)
    {
        u->IncAngleMax = u->IncAngleMax - u->IncAngle;
        u->Angle = u->Angle + u->CMDDIR * HALL1.IncAngle;
    }
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
    HallValue = (uint8_t)(HallA * 1 + HallB * 2 + HallC * 4);

    return HallValue;
}

/****************************************************************
    Function: HALLCheck
****************************************************************/
void HALLCheck(HALLType *u)
{
    if ((u->RunHallValue == 0) || (u->RunHallValue == 7))
    {
        FAULT.bit.HallFlag = 1;
    }
}
