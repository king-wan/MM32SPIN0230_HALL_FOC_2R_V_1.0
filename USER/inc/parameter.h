#ifndef __PARAMETER_H_
#define __PARAMETER_H_

#define SingleShunt

/* System setting */
#define SYS_REFV                      5.0
#define SYSCLK_HSI_60MHz             60000000

#define PWMFREQ                      16000
#define PWMPERIOD                    (SYSCLK_HSI_60MHz / PWMFREQ / 2)
#define DEAD_TIME                    60

/* Current offset calibration and runtime filtering */
#define CURRENT_OFFSET_CAL_SAMPLES   128U
#define CURRENT_OFFSET_TRACK_ENABLE  1U
#define CURRENT_RUNTIME_FILTER_ENABLE 1U
#define CURRENT_FILTER_SHIFT         1U
#define CURRENT_OFFSET_TRACK_SHIFT   7U
#define CURRENT_OFFSET_TRACK_MAX_RPM 80
#define CURRENT_OFFSET_TRACK_TH_Q3   480

#define TIM2_PSC_LOAD                ((u16)(SYSCLK_HSI_60MHz / 1000000 - 1))
#define TIM2_PRIOD                   99999

#endif
