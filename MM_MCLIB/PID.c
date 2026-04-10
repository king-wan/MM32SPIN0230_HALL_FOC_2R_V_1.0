/*-------------------- Includes -----------------------*/
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

/****************************************************************
    Function: InitPI
****************************************************************/
void InitPI(void)
{
    CurID.qKp = 9000;
    CurID.qKi = 200;
    CurID.qOutMax = 4000;
    CurID.qOutMin = -4000;
    CurID.qdSum = 0;
    CurID.qInMeas = 0;
    CurID.qOut = 0;

    CurIQ.qKp = 9000;
    CurIQ.qKi = 200;
    CurIQ.qOutMax = 30000;
    CurIQ.qOutMin = -30000;
    CurIQ.qdSum = 0;
    CurIQ.qInMeas = 0;
    CurIQ.qOut = 0;

    Speed.qKp = 5800;
    Speed.qKi = 65;
    Speed.qOutMax = IshuntGain * 5;
#if POSITION_LOOP_ENABLE
    Speed.qOutMin = -(IshuntGain * 5);
#else
    Speed.qOutMin = 0;
#endif
    Speed.qdSum = 0;
    Speed.qInMeas = 0;
    Speed.qOut = 0;

    PLL.qKp = 2000;
    PLL.qKi = 10;
    PLL.qOutMax = 500;
    PLL.qOutMin = -500;
    PLL.qdSum = 0;
    PLL.qInRef = 0;
    PLL.qInMeas = 0;
    PLL.qOut = 0;

    Position.qKp = POSITION_KP_Q15;
    Position.qKi = POSITION_KI_Q15;
    Position.qOutMax = POSITION_SPEED_MAX_RPM;
    Position.qOutMin = -POSITION_SPEED_MAX_RPM;
    Position.qdSum = 0;
    Position.qInRef = 0;
    Position.qInMeas = 0;
    Position.qOut = 0;
}

/****************************************************************
    Function: CalcPI
****************************************************************/
void CalcPI(tPIParm *pParm)
{
    int32_t error;
    int32_t up;
    int32_t ui;
    int32_t out;
    int32_t daux;
    int32_t i_limit_max;
    int32_t i_limit_min;

    error = (int32_t)pParm->qInRef - (int32_t)pParm->qInMeas;

    /* Proportional term (Q15 gain). */
    up = ((int32_t)pParm->qKp * error) >> 15;

    if (pParm->qKi == 0)
    {
        pParm->qdSum = 0;
        ui = 0;
    }
    else
    {
        /* Integral stored as pure accumulated error with explicit anti-windup limits. */
        i_limit_max = ((int32_t)pParm->qOutMax << 15) / (int32_t)pParm->qKi;
        i_limit_min = ((int32_t)pParm->qOutMin << 15) / (int32_t)pParm->qKi;

        daux = pParm->qdSum + error;
        if (daux > i_limit_max)
        {
            daux = i_limit_max;
        }
        else if (daux < i_limit_min)
        {
            daux = i_limit_min;
        }

        pParm->qdSum = daux;
        ui = (pParm->qdSum * (int32_t)pParm->qKi) >> 15;
    }

    out = up + ui;
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


