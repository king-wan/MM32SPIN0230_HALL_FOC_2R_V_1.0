/*-------------------- Includes -----------------------*/
#include <stdint.h>
#include "PID.h"
#include "MC_Drive.h"
#include "user_function.h"
#include "hall_tune.h"

/*------------------- Private variables ---------------*/
tPIParm CurID;
tPIParm CurIQ;
tPIParm Speed;
tPIParm PLL;
tPIParm Position;

#define PID_Q15_DIVISOR    32768L

static void PI_UpdateIntegralLimit(tPIParm *pParm, uint8_t rescale_integral);
static void PI_Configure(tPIParm *pParm, int16_t kp, int16_t ki, int16_t out_min, int16_t out_max);

static void PI_UpdateIntegralLimit(tPIParm *pParm, uint8_t rescale_integral)
{
    int32_t old_sum = pParm->qdSum;

    if ((pParm->qKi == 0) || (pParm->qKiDiv == 0))
    {
        pParm->qdSum = 0;
        pParm->qdSumMax = 0;
        pParm->qdSumMin = 0;
    }
    else
    {
        if (rescale_integral && (pParm->qKiPrev != 0) && (pParm->qKiPrev != pParm->qKi))
        {
            old_sum = (int32_t)(((int64_t)old_sum * (int64_t)pParm->qKiPrev) / (int64_t)pParm->qKi);
        }

        pParm->qdSumMax = ((int32_t)pParm->qOutMax * (int32_t)pParm->qKiDiv) / (int32_t)pParm->qKi;
        pParm->qdSumMin = ((int32_t)pParm->qOutMin * (int32_t)pParm->qKiDiv) / (int32_t)pParm->qKi;

        if (old_sum > pParm->qdSumMax)
        {
            old_sum = pParm->qdSumMax;
        }
        else if (old_sum < pParm->qdSumMin)
        {
            old_sum = pParm->qdSumMin;
        }

        pParm->qdSum = old_sum;
    }

    pParm->qKiPrev = pParm->qKi;
    pParm->qOutMaxPrev = pParm->qOutMax;
    pParm->qOutMinPrev = pParm->qOutMin;
}

static void PI_Configure(tPIParm *pParm, int16_t kp, int16_t ki, int16_t out_min, int16_t out_max)
{
    pParm->qKp = kp;
    pParm->qKpDiv = PID_Q15_DIVISOR;
    pParm->qKi = ki;
    pParm->qKiDiv = PID_Q15_DIVISOR;
    pParm->qKd = 0;
    pParm->qKdDiv = PID_Q15_DIVISOR;
    pParm->qOutMax = out_max;
    pParm->qOutMin = out_min;
    pParm->qInRef = 0;
    pParm->qInMeas = 0;
    pParm->qOut = 0;
    pParm->qdSum = 0;
    pParm->qPrevErr = 0;
    pParm->qKiPrev = ki;
    pParm->qOutMaxPrev = out_max;
    pParm->qOutMinPrev = out_min;

    PI_UpdateIntegralLimit(pParm, 0);
}

/****************************************************************
    Function: InitPI
****************************************************************/
void InitPI(void)
{
    PI_Configure(&CurID, 9000, 200, -4000, 4000);
    PI_Configure(&CurIQ, 9000, 200, -30000, 30000);

#if POSITION_LOOP_ENABLE
    PI_Configure(&Speed, 5800, 65, -SPEED_LOOP_MAX_CURRENT_Q15, SPEED_LOOP_MAX_CURRENT_Q15);
#else
    PI_Configure(&Speed, 5800, 65, 0, SPEED_LOOP_MAX_CURRENT_Q15);
#endif

    PI_Configure(&PLL, 2000, 10, -500, 500);
    PI_Configure(&Position, POSITION_KP_Q15, POSITION_KI_Q15, -POSITION_SPEED_MAX_RPM, POSITION_SPEED_MAX_RPM);
}

/****************************************************************
    Function: CalcPI
****************************************************************/
void CalcPI(tPIParm *pParm)
{
    int32_t error;
    int32_t up;
    int32_t ui;
    int32_t ud;
    int32_t out;
    int32_t daux;

    if ((pParm->qKi != pParm->qKiPrev) ||
        (pParm->qOutMax != pParm->qOutMaxPrev) ||
        (pParm->qOutMin != pParm->qOutMinPrev))
    {
        PI_UpdateIntegralLimit(pParm, 1);
    }

    error = (int32_t)pParm->qInRef - (int32_t)pParm->qInMeas;

    if (pParm->qKpDiv == 0)
    {
        pParm->qKpDiv = PID_Q15_DIVISOR;
    }
    if (pParm->qKiDiv == 0)
    {
        pParm->qKiDiv = PID_Q15_DIVISOR;
    }
    if (pParm->qKdDiv == 0)
    {
        pParm->qKdDiv = PID_Q15_DIVISOR;
    }

    up = ((int32_t)pParm->qKp * error) / (int32_t)pParm->qKpDiv;

    if ((pParm->qKi == 0) || (pParm->qKiDiv == 0))
    {
        pParm->qdSum = 0;
        ui = 0;
    }
    else
    {
        daux = pParm->qdSum + error;
        if (daux > pParm->qdSumMax)
        {
            daux = pParm->qdSumMax;
        }
        else if (daux < pParm->qdSumMin)
        {
            daux = pParm->qdSumMin;
        }

        pParm->qdSum = daux;
        ui = (pParm->qdSum * (int32_t)pParm->qKi) / (int32_t)pParm->qKiDiv;
    }

    if ((pParm->qKd != 0) && (pParm->qKdDiv != 0))
    {
        ud = ((int32_t)pParm->qKd * (error - pParm->qPrevErr)) / (int32_t)pParm->qKdDiv;
    }
    else
    {
        ud = 0;
    }
    pParm->qPrevErr = error;

    out = up + ui + ud;
    if (out > pParm->qOutMax)
    {
        out = pParm->qOutMax;
    }
    else if (out < pParm->qOutMin)
    {
        out = pParm->qOutMin;
    }

    pParm->qOut = (int16_t)out;
}


