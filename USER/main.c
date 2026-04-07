/**
 * @file     main.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides the main functions and test samples.
 */

#define _MAIN_C_

#include "main.h"
#include "drv_inc.h"
#include "board.h"
#include "user_function.h"
#include "MC_Drive.h"
#include "Diagnose.h"
#include "FOC_Math.h"
#include "HallHandle.h"
#include "PID.h"
#include <stdio.h>

int16_t M1FaultID, M1FaultID_Record;

#define SPEED_START_THRESHOLD    120
#define SPEED_STOP_THRESHOLD      80
#define HALL_DEBUG_PRINT          0
#define HALL_EDGE_PRINT_ONLY      1
#define HALL_START_STOP_PRINT     1
#define STOP_REPORT_SPEED_TH      25
#define STOP_REPORT_STABLE_CNT    20
#define STOP_REPORT_DELAY_CNT     120
#define STOP_REPORT_TIMEOUT_CNT   400
#define CMD_EVENT_DEBOUNCE_CNT     6
#define ZERO_REF_CLAMP_TH         40
#define RUN200_PERIOD_CNT          40
#define RUN_MIN_REF              140
#define RUN_MIN_OUT               45

int main(void)
{
    Systick_Init(SystemCoreClock / 1000);
    Systick_Delay(200);

    Init_Parameter();
    Bsp_Gpio_Init();
    Peripheral_Init();
    Interrupt_Init();

    while (1)
    {
        Board_USART_DMA_Task();
        IWDG_RELOAD_COUNT();

        if (TIMFlag.PWMIn == 1)
        {
            TIMFlag.PWMIn = 0;

            if (ADC_Structure.IBusInput <= 0)
            {
                ADC_Structure.IBusInput = 0;
                ADC_Structure.IBusAvrg = 0;
            }
            else
            {
                ADC_Structure.IBusInput = (IsumGain * ADC_Structure.IBusInput) >> 12;
                ADC_Structure.IBusAvrg = (ADC_Structure.IBusAvrg * 7 + ADC_Structure.IBusInput) >> 3;
            }

            Diagnose_IBUS_ADC(ADC_Structure.IBusAvrg);
        }

        if (TIMFlag.Delay5ms == 1)
        {
            static uint8_t s_speed_enable = 0;
            static uint8_t s_prev_outen = 0;
            static uint8_t s_wait_stop_report = 0;
            static uint8_t s_stop_stable_cnt = 0;
            static uint16_t s_stop_delay_cnt = 0;
            static uint8_t s_stop_last_hall = 0;
            static uint8_t s_cmd_raw_prev = 0;
            static uint8_t s_cmd_filt = 0;
            static uint8_t s_cmd_stable_cnt = 0;
            static uint16_t s_run200_cnt = 0;

            TIMFlag.Delay5ms = 0;

            CalcNormalization(ADC_Structure.SPEED, &RP);

            /* minimum start speed and stop hysteresis */
            if (RP.OutEn == 0)
            {
                s_speed_enable = 0;
                RP.Out = 0;
            }
            else if (s_speed_enable == 0)
            {
                if (RP.Out >= SPEED_START_THRESHOLD)
                {
                    s_speed_enable = 1;
                }
                else
                {
                    RP.Out = 0;
                }
            }
            else
            {
                if (RP.Out <= SPEED_STOP_THRESHOLD)
                {
                    s_speed_enable = 0;
                    RP.Out = 0;
                }
            }

            RPValue.Dest = RP.Out;
            LoopCmp_Cal(&RPValue);
            if (s_speed_enable == 0U)
            {
                /* Commanded stop: force speed reference to zero immediately. */
                RPValue.Dest = 0;
                RPValue.Act = 0;
            }
            else if (RPValue.Act < RUN_MIN_REF)
            {
                /* Avoid weak-command stall right after start. */
                RPValue.Act = RUN_MIN_REF;
            }
            else if ((RPValue.Dest == 0) && (RPValue.Act <= ZERO_REF_CLAMP_TH))
            {
                RPValue.Act = 0;
            }

            SpeedFdk.NewData = HALL1.SpeedTemp;
            MovingAvgCal(&SpeedFdk);

            Speed.qInRef = RPValue.Act;
            Speed.qInMeas = SpeedFdk.Out;
            CalcPI(&Speed);
            if ((s_speed_enable != 0U) && (Speed.qOut < RUN_MIN_OUT))
            {
                Speed.qOut = RUN_MIN_OUT;
            }
            if (Speed.qOut < 0)
            {
                /* Forward-only mode: block reverse torque near stop. */
                Speed.qOut = 0;
                Speed.qdSum = 0;
            }

            if (SpeedFdk.Out <= 500)
            {
                IdRef = 0;
            }
            else if (SpeedFdk.Out >= 800)
            {
                IdRef = -350;
            }
            else
            {
                IdRef = -((int32_t)(SpeedFdk.Out - 500) * 350 / 300);
            }

            Diagnose_VBUS_ADC(ADC_Structure.VBusInput);

#if HALL_START_STOP_PRINT
            uint8_t cmd_raw = s_speed_enable;
            uint8_t cmd_active;
            if (cmd_raw == s_cmd_raw_prev)
            {
                if (s_cmd_stable_cnt < 250U) { s_cmd_stable_cnt++; }
            }
            else
            {
                s_cmd_raw_prev = cmd_raw;
                s_cmd_stable_cnt = 0;
            }

            if (s_cmd_stable_cnt >= CMD_EVENT_DEBOUNCE_CNT)
            {
                s_cmd_filt = cmd_raw;
            }
            cmd_active = s_cmd_filt;

            if ((cmd_active != 0U) && (s_prev_outen == 0U))
            {
                char evt_buf[96];
                snprintf(evt_buf, sizeof(evt_buf),
                         "START hall=%u ang=%d spd=%d\r\n",
                         (unsigned int)HALL_ReadHallPorts(),
                         (int)HALL1.Angle,
                         (int)HALL1.SpeedTemp);
                Board_USART_SendString(evt_buf);
                s_wait_stop_report = 0;
                s_stop_stable_cnt = 0;
                s_stop_delay_cnt = 0;
            }
            else if ((cmd_active == 0U) && (s_prev_outen != 0U))
            {
                s_wait_stop_report = 1;
                s_stop_stable_cnt = 0;
                s_stop_delay_cnt = 0;
                s_stop_last_hall = HALL_ReadHallPorts();
            }

            if (cmd_active != 0U)
            {
                s_run200_cnt++;
                if (s_run200_cnt >= RUN200_PERIOD_CNT)
                {
                    char evt_buf[96];
                    snprintf(evt_buf, sizeof(evt_buf),
                             "RUN200 hall=%u spd=%d fdk=%d ref=%d out=%d\r\n",
                             (unsigned int)HALL_ReadHallPorts(),
                             (int)HALL1.SpeedTemp,
                             (int)SpeedFdk.Out,
                             (int)RPValue.Act,
                             (int)Speed.qOut);
                    Board_USART_SendString(evt_buf);
                    s_run200_cnt = 0;
                }
            }
            else
            {
                s_run200_cnt = 0;
            }

            if (s_wait_stop_report != 0)
            {
                uint8_t hall_now = HALL_ReadHallPorts();
                s_stop_delay_cnt++;

                if (hall_now == s_stop_last_hall)
                {
                    if (s_stop_stable_cnt < 250U) { s_stop_stable_cnt++; }
                }
                else
                {
                    s_stop_last_hall = hall_now;
                    s_stop_stable_cnt = 0;
                }

                if ((s_stop_delay_cnt >= STOP_REPORT_DELAY_CNT) &&
                    (s_stop_stable_cnt >= STOP_REPORT_STABLE_CNT) &&
                    (SpeedFdk.Out <= STOP_REPORT_SPEED_TH))
                {
                    char evt_buf[96];
                    snprintf(evt_buf, sizeof(evt_buf),
                             "STOP  hall=%u ang=%d spd=%d\r\n",
                             (unsigned int)hall_now,
                             (int)HALL1.Angle,
                             (int)HALL1.SpeedTemp);
                    Board_USART_SendString(evt_buf);
                    s_wait_stop_report = 0;
                    s_stop_stable_cnt = 0;
                    s_stop_delay_cnt = 0;
                }
                else if (s_stop_delay_cnt >= STOP_REPORT_TIMEOUT_CNT)
                {
                    char evt_buf[96];
                    snprintf(evt_buf, sizeof(evt_buf),
                             "STOP_TMO hall=%u ang=%d spd=%d\r\n",
                             (unsigned int)hall_now,
                             (int)HALL1.Angle,
                             (int)HALL1.SpeedTemp);
                    Board_USART_SendString(evt_buf);
                    s_wait_stop_report = 0;
                    s_stop_stable_cnt = 0;
                    s_stop_delay_cnt = 0;
                }
            }

            s_prev_outen = cmd_active;
#endif
        }

#if HALL_DEBUG_PRINT
#if HALL_EDGE_PRINT_ONLY
        {
            while (g_hall_edge_count != 0)
            {
                char dbg_buf[128];
                uint8_t old_hall;
                uint8_t new_hall;

                __disable_irq();
                old_hall = g_hall_edge_old_buf[g_hall_edge_tail];
                new_hall = g_hall_edge_new_buf[g_hall_edge_tail];
                g_hall_edge_tail = (uint8_t)((g_hall_edge_tail + 1) & 0x1F);
                g_hall_edge_count--;
                __enable_irq();

                snprintf(dbg_buf, sizeof(dbg_buf),
                         "edge %u->%u spd=%d fdk=%d ang=%d ref=%d out=%d\r\n",
                         (unsigned int)old_hall,
                         (unsigned int)new_hall,
                         (int)HALL1.SpeedTemp,
                         (int)SpeedFdk.Out,
                         (int)HALL1.Angle,
                         (int)RPValue.Act,
                         (int)Speed.qOut);
                Board_USART_SendString(dbg_buf);
            }
        }
#else
        if (TIMFlag.Delay200ms == 1)
        {
            char dbg_buf[128];
            TIMFlag.Delay200ms = 0;

            snprintf(dbg_buf, sizeof(dbg_buf),
                     "hall=%u spd=%d ang=%d ref=%d out=%d raw=%u\r\n",
                     (unsigned int)HALL1.RunHallValue,
                     (int)HALL1.SpeedTemp,
                     (int)HALL1.Angle,
                     (int)RPValue.Act,
                     (int)Speed.qOut,
                     (unsigned int)HALL_ReadHallPorts());
            Board_USART_SendString(dbg_buf);
        }
#endif
#endif
    }
}

