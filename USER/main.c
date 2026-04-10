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
#include "hall_tune.h"
#include <stdio.h>

int16_t M1FaultID, M1FaultID_Record;

#define HALL_DEBUG_PRINT          HALL_DEBUG_ENABLE
#define HALL_EDGE_PRINT_ONLY      HALL_DEBUG_EDGE_ONLY
#define HALL_START_STOP_PRINT     HALL_DEBUG_START_STOP
#define STOP_REPORT_SPEED_TH      25
#define STOP_REPORT_STABLE_CNT    20
#define STOP_REPORT_DELAY_CNT     120
#define STOP_REPORT_TIMEOUT_CNT   400
#define CMD_EVENT_DEBOUNCE_CNT     6
#define RUN200_PERIOD_CNT          40

extern tPIParm Position;
extern uint8_t PositionHoldEnable;
extern int16_t PositionHoldTargetAngle;
extern int16_t PositionHoldIq;
extern int16_t HallAngleOffset;

static int16_t g_pos_ref_q15 = POSITION_REF_Q15_DEFAULT;
static int16_t g_pos_fdb_q15 = 0;

static const uint8_t k_pos_hall_seq[POSITION_SECTOR_COUNT] = {1, 3, 2, 6, 4, 5};
static uint8_t g_pos_target_sector = 0;
static uint8_t g_pos_current_sector = 0;

static int16_t WrapAngleErrQ15(int16_t target, int16_t angle)
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

static uint16_t Abs16(int16_t value)
{
    return (value >= 0) ? (uint16_t)value : (uint16_t)(-value);
}

static int16_t PotToPositionQ15(uint16_t adc)
{
    int32_t span = (int32_t)POSITION_POT_MAX_ADC - (int32_t)POSITION_POT_MIN_ADC;
    int32_t norm;

    if (adc <= POSITION_POT_MIN_ADC)
    {
        return -32768;
    }
    if (adc >= POSITION_POT_MAX_ADC)
    {
        return 32767;
    }

    if (span <= 0)
    {
        return 0;
    }

    norm = ((int32_t)adc - (int32_t)POSITION_POT_MIN_ADC) * 65535 / span;
    return (int16_t)(norm - 32768);
}

static uint8_t PotToSectorIndex(uint16_t adc)
{
    int32_t span = (int32_t)POSITION_POT_MAX_ADC - (int32_t)POSITION_POT_MIN_ADC + 1;
    int32_t scaled;

    if (adc <= POSITION_POT_MIN_ADC)
    {
        return 0U;
    }
    if (adc >= POSITION_POT_MAX_ADC)
    {
        return (uint8_t)(POSITION_SECTOR_COUNT - 1U);
    }
    if (span <= 0)
    {
        return 0U;
    }

    scaled = ((int32_t)adc - (int32_t)POSITION_POT_MIN_ADC) * POSITION_SECTOR_COUNT / span;
    if (scaled >= POSITION_SECTOR_COUNT)
    {
        scaled = POSITION_SECTOR_COUNT - 1;
    }
    return (uint8_t)scaled;
}

static uint8_t HallStateToIndex(uint8_t hall)
{
    uint8_t i;

    for (i = 0; i < POSITION_SECTOR_COUNT; i++)
    {
        if (k_pos_hall_seq[i] == hall)
        {
            return i;
        }
    }
    return 0U;
}

static int16_t GetSectorCenterAngleQ15(uint8_t hall, int8_t dir)
{
    int32_t angle;

    if ((hall == 0U) || (hall == 7U))
    {
        hall = 1U;
    }

    if (dir < 0)
    {
        angle = (int32_t)HALL1.CCWAngleTab[hall] + 5460 + HallAngleOffset;
    }
    else
    {
        angle = (int32_t)HALL1.CWAngleTab[hall] + 5460 + HallAngleOffset;
    }

    if (angle > 32767)
    {
        angle -= 65536;
    }
    else if (angle < -32768)
    {
        angle += 65536;
    }
    return (int16_t)angle;
}

static void Hall_DebugSnapshot(char *buf, size_t buf_size, const char *tag)
{
    snprintf(buf, buf_size,
             "%s raw=%u run=%u pre=%u dir=%d hs=%u st=%u spd=%d fdk=%d ang=%d ctr=%d lo=%d hi=%d dpp=%d ref=%d out=%d pref=%d pfdb=%d adc=%u pot=%d sec=%u cur=%u\r\n",
             tag,
             (unsigned int)HALL_ReadHallPorts(),
             (unsigned int)HALL1.RunHallValue,
             (unsigned int)HALL1.PreHallValue,
             (int)HALL1.EdgeDir,
             (unsigned int)HALL1.UseHalfSector,
             (unsigned int)HALL1.HallState,
             (int)HALL1.SpeedTemp,
             (int)SpeedFdk.Out,
             (int)HALL1.Angle,
             (int)HALL1.AngleCenter,
             (int)HALL1.AngleLowLimit,
             (int)HALL1.AngleHighLimit,
             (int)HALL1.IncAngle,
             (int)RPValue.Act,
             (int)Speed.qOut,
             (int)g_pos_ref_q15,
             (int)g_pos_fdb_q15,
             (unsigned int)ADC_Structure.SPEED,
             (int)RP.Out,
             (unsigned int)g_pos_target_sector,
             (unsigned int)g_pos_current_sector);
}

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
            static uint8_t s_pos_servo_active = 0;
            static uint8_t s_prev_outen = 0;
            static uint8_t s_wait_stop_report = 0;
            static uint8_t s_stop_stable_cnt = 0;
            static uint16_t s_stop_delay_cnt = 0;
            static uint8_t s_stop_last_hall = 0;
            static uint8_t s_cmd_raw_prev = 0;
            static uint8_t s_cmd_filt = 0;
            static uint8_t s_cmd_stable_cnt = 0;
            static uint16_t s_run200_cnt = 0;
            static int16_t s_pos_ref_filt_q15 = POSITION_REF_Q15_DEFAULT;
            static uint8_t s_coarse_goal_idx = 0;
            static uint8_t s_coarse_step_idx = 0;
            static int8_t s_coarse_dir = 1;
            static uint8_t s_coarse_step_hold_cnt = 0;
            int16_t pos_err_q15 = 0;
            uint16_t abs_pos_err = 0;

            TIMFlag.Delay5ms = 0;

            CalcNormalization(ADC_Structure.SPEED, &RP);

#if POSITION_LOOP_ENABLE
            g_pos_fdb_q15 = HALL1.Angle;
            if (RP.OutEn == 0U)
            {
                s_speed_enable = 0;
                s_pos_servo_active = 0;
                s_coarse_goal_idx = POSITION_SECTOR_COUNT;
                s_coarse_step_idx = 0;
                s_coarse_dir = HALL1.CMDDIR;
                s_coarse_step_hold_cnt = 0;
                PositionHoldEnable = 0;
                RP.Out = 0;
                g_pos_ref_q15 = g_pos_fdb_q15;
                s_pos_ref_filt_q15 = g_pos_ref_q15;
                g_pos_target_sector = 0;
                g_pos_current_sector = 0;
                pos_err_q15 = 0;
                abs_pos_err = 0;
                Position.qdSum = 0;
                Position.qOut = 0;
                Speed.qdSum = 0;
                Speed.qOut = 0;
            }
            else
            {
#if POSITION_CMD_SRC_POT
#if POSITION_COARSE_MODE
                uint8_t hall_now = HALL1.RunHallValue;
                uint8_t curr_idx;
                uint8_t tgt_idx = PotToSectorIndex((uint16_t)ADC_Structure.SPEED);

                if ((hall_now == 0U) || (hall_now == 7U))
                {
                    hall_now = HALL_ReadHallPorts();
                }
                curr_idx = HallStateToIndex(hall_now);
                g_pos_current_sector = (uint8_t)(curr_idx + 1U);
                g_pos_target_sector = (uint8_t)(tgt_idx + 1U);
                HALL1.CMDDIR = POSITION_HOLD_DIR;
                s_coarse_dir = POSITION_HOLD_DIR;
                s_coarse_goal_idx = tgt_idx;
                s_coarse_step_idx = tgt_idx;
                s_coarse_step_hold_cnt = 0;
                g_pos_fdb_q15 = GetSectorCenterAngleQ15(k_pos_hall_seq[curr_idx], POSITION_HOLD_DIR);
                g_pos_ref_q15 = GetSectorCenterAngleQ15(k_pos_hall_seq[tgt_idx], POSITION_HOLD_DIR);
                s_pos_ref_filt_q15 = g_pos_ref_q15;
#else
                int16_t pos_ref_raw_q15 = PotToPositionQ15((uint16_t)ADC_Structure.SPEED);
                s_pos_ref_filt_q15 = (int16_t)(s_pos_ref_filt_q15 +
                    ((int32_t)pos_ref_raw_q15 - (int32_t)s_pos_ref_filt_q15) / (1 << POSITION_REF_FILTER_SHIFT));
                g_pos_ref_q15 = s_pos_ref_filt_q15;
                g_pos_target_sector = 0;
#endif
#endif
                pos_err_q15 = WrapAngleErrQ15(g_pos_ref_q15, g_pos_fdb_q15);
                abs_pos_err = Abs16(pos_err_q15);

                if (s_pos_servo_active == 0U)
                {
                    if (abs_pos_err >= POSITION_ERR_RELEASE_Q15)
                    {
                        s_pos_servo_active = 1;
                    }
                }
                else
                {
                    if (abs_pos_err <= POSITION_ERR_DEAD_Q15)
                    {
                        s_pos_servo_active = 0;
                    }
                }

                s_speed_enable = (s_pos_servo_active != 0U) ? 1U : 0U;
            }
            PositionHoldEnable = ((RP.OutEn != 0U) &&
                                  (s_pos_servo_active == 0U) &&
                                  (g_pos_current_sector != 0U) &&
                                  (g_pos_current_sector == g_pos_target_sector)) ? 1U : 0U;
            PositionHoldTargetAngle = g_pos_ref_q15;
            PositionHoldIq = POSITION_HOLD_IQ_Q15;

            Position.qInRef = pos_err_q15;
            Position.qInMeas = 0;
            CalcPI(&Position);
            RP.Out = (s_speed_enable != 0U) ? Position.qOut : 0;
#endif

#if !POSITION_LOOP_ENABLE
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
#endif

#if POSITION_LOOP_ENABLE
            RPValue.Max = POSITION_SPEED_MAX_RPM;
            RPValue.Min = -POSITION_SPEED_MAX_RPM;
#endif
            RPValue.Dest = RP.Out;
            LoopCmp_Cal(&RPValue);
            if (s_speed_enable == 0U)
            {
                /* Commanded stop: force speed reference to zero immediately. */
                RPValue.Dest = 0;
                RPValue.Act = 0;
            }
#if POSITION_LOOP_ENABLE
            else if ((RPValue.Act > 0) && (RPValue.Act < POSITION_SPEED_MIN_RPM))
            {
                /* Hall position mode needs a minimum crawl speed to break static friction. */
                RPValue.Act = POSITION_SPEED_MIN_RPM;
            }
            else if ((RPValue.Act < 0) && (RPValue.Act > -POSITION_SPEED_MIN_RPM))
            {
                RPValue.Act = -POSITION_SPEED_MIN_RPM;
            }
#else
            else if (RPValue.Act < RUN_MIN_REF_RPM)
            {
                /* Avoid weak-command stall right after start. */
                RPValue.Act = RUN_MIN_REF_RPM;
            }
#endif
            else if ((RPValue.Dest == 0) && (Abs16((int16_t)RPValue.Act) <= ZERO_REF_CLAMP_TH))
            {
                RPValue.Act = 0;
            }

            SpeedFdk.NewData = HALL1.SpeedTemp;
            MovingAvgCal(&SpeedFdk);

            Speed.qInRef = RPValue.Act;
            Speed.qInMeas = SpeedFdk.Out;
            CalcPI(&Speed);
#if !POSITION_LOOP_ENABLE
            if ((s_speed_enable != 0U) && (Speed.qOut < RUN_MIN_OUT_Q15))
            {
                Speed.qOut = RUN_MIN_OUT_Q15;
            }
            if (Speed.qOut < 0)
            {
                /* Forward-only mode: block reverse torque near stop. */
                Speed.qOut = 0;
                Speed.qdSum = 0;
            }
#else
            if (s_pos_servo_active != 0U)
            {
                if ((Speed.qOut > 0) && (Speed.qOut < POSITION_TORQUE_MIN_Q15))
                {
                    Speed.qOut = POSITION_TORQUE_MIN_Q15;
                }
                else if ((Speed.qOut < 0) && (Speed.qOut > -POSITION_TORQUE_MIN_Q15))
                {
                    Speed.qOut = -POSITION_TORQUE_MIN_Q15;
                }
            }
#endif

            if (SpeedFdk.Out <= FIELD_WEAKEN_START_RPM)
            {
                IdRef = 0;
            }
            else if (SpeedFdk.Out >= FIELD_WEAKEN_FULL_RPM)
            {
                IdRef = -FIELD_WEAKEN_MAX_ID_Q15;
            }
            else
            {
                IdRef = -((int32_t)(SpeedFdk.Out - FIELD_WEAKEN_START_RPM) *
                          FIELD_WEAKEN_MAX_ID_Q15 /
                          (FIELD_WEAKEN_FULL_RPM - FIELD_WEAKEN_START_RPM));
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
                char evt_buf[192];
                Hall_DebugSnapshot(evt_buf, sizeof(evt_buf), "START");
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
                    char evt_buf[192];
                    Hall_DebugSnapshot(evt_buf, sizeof(evt_buf), "RUN200");
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
                    char evt_buf[192];
                    (void)hall_now;
                    Hall_DebugSnapshot(evt_buf, sizeof(evt_buf), "STOP");
                    Board_USART_SendString(evt_buf);
                    s_wait_stop_report = 0;
                    s_stop_stable_cnt = 0;
                    s_stop_delay_cnt = 0;
                }
                else if (s_stop_delay_cnt >= STOP_REPORT_TIMEOUT_CNT)
                {
                    char evt_buf[192];
                    (void)hall_now;
                    Hall_DebugSnapshot(evt_buf, sizeof(evt_buf), "STOP_TMO");
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
                char dbg_buf[192];
                uint8_t old_hall;
                uint8_t new_hall;

                __disable_irq();
                old_hall = g_hall_edge_old_buf[g_hall_edge_tail];
                new_hall = g_hall_edge_new_buf[g_hall_edge_tail];
                g_hall_edge_tail = (uint8_t)((g_hall_edge_tail + 1) & 0x1F);
                g_hall_edge_count--;
                __enable_irq();

                snprintf(dbg_buf, sizeof(dbg_buf),
                         "edge %u->%u dir=%d hs=%u st=%u spd=%d fdk=%d ang=%d ctr=%d lo=%d hi=%d dpp=%d ref=%d out=%d\r\n",
                         (unsigned int)old_hall,
                         (unsigned int)new_hall,
                         (int)HALL1.EdgeDir,
                         (unsigned int)HALL1.UseHalfSector,
                         (unsigned int)HALL1.HallState,
                         (int)HALL1.SpeedTemp,
                         (int)SpeedFdk.Out,
                         (int)HALL1.Angle,
                         (int)HALL1.AngleCenter,
                         (int)HALL1.AngleLowLimit,
                         (int)HALL1.AngleHighLimit,
                         (int)HALL1.IncAngle,
                         (int)RPValue.Act,
                         (int)Speed.qOut);
                Board_USART_SendString(dbg_buf);
            }
        }
#else
        if (TIMFlag.Delay200ms == 1)
        {
            char dbg_buf[192];
            TIMFlag.Delay200ms = 0;

            Hall_DebugSnapshot(dbg_buf, sizeof(dbg_buf), "HALL200");
            Board_USART_SendString(dbg_buf);
        }
#endif
#endif
    }
}

