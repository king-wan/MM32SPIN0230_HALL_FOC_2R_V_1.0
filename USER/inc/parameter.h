#ifndef __PARAMETER_H_
#define __PARAMETER_H_

/*
 * System/control common parameters
 * --------------------------------
 * motor_profile.h: motor-specific data and tuning.
 * user_function.h: board analog front-end and protection thresholds.
 * parameter.h: controller timing, PWM base settings, and ADC current filter
 * behavior shared by the whole project.
 *
 * File role:
 * - Edit this file only when the control platform itself changes.
 * - Typical reasons: PWM frequency needs to change, ADC filtering strategy
 *   changes, or Hall timing base needs adjustment.
 * - For ordinary motor replacement, this file is usually not the first place
 *   to touch.
 */

#define SingleShunt                   /* Build option kept from the original project. */

/* System setting */
#define SYS_REFV                      5.0       /* ADC/reference voltage, unit: V. */
#define SYSCLK_HSI_60MHz             60000000  /* Main system clock, unit: Hz. */

/* PWM base timing */
#define PWMFREQ                      16000     /* PWM switching frequency, unit: Hz. */
#define PWMPERIOD                    (SYSCLK_HSI_60MHz / PWMFREQ / 2) /* Center-aligned PWM half-period in timer counts. */
#define DEAD_TIME                    60        /* PWM dead time register setting, unit: timer counts. */

/* Current offset calibration and runtime filtering */
#define CURRENT_OFFSET_CAL_SAMPLES    128U     /* Number of startup samples used for current zero-offset calibration. */
#define CURRENT_OFFSET_TRACK_ENABLE   1U       /* Enable idle-time current offset tracking: 1=on, 0=off. */
#define CURRENT_RUNTIME_FILTER_ENABLE 1U       /* Enable runtime current IIR filtering in RUNSTATE: 1=on, 0=off. */
#define CURRENT_FILTER_SHIFT          1U       /* Runtime current filter strength, larger means heavier filtering. */
#define CURRENT_OFFSET_TRACK_SHIFT    7U       /* Idle offset tracking speed, larger means slower adaptation. */
#define CURRENT_OFFSET_TRACK_MAX_RPM  80       /* Offset tracking allowed only below this speed, unit: 10 rpm. */
#define CURRENT_OFFSET_TRACK_TH_Q3    480      /* Offset tracking allowed only when raw current is below this q3 threshold. */

/* TIM2 Hall timing base */
#define TIM2_PSC_LOAD                ((u16)(SYSCLK_HSI_60MHz / 1000000 - 1)) /* TIM2 prescaler for 1 MHz counter clock. */
#define TIM2_PRIOD                   99999     /* TIM2 auto-reload value for Hall period capture window. */

#endif
