/** 
 * @file     drv_pwm.h
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the driver functions prototypes for the PWM.
 *
 * @attention
 *
 * THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 * CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 * TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 * HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 * CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 *
 * <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
 */

/** Define to prevent recursive inclusion */
#ifndef __DRV_PWM_H
#define __DRV_PWM_H

/** Files includes */
#include "mm32_device.h"
#include "HAL_conf.h"

/** 
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/** 
 * @addtogroup Drv_PWM
 * @{
 */


#define SET_CCR1_VAL(Value)         WRITE_REG(TIM1->CCR1, Value)
#define SET_CCR2_VAL(Value)         WRITE_REG(TIM1->CCR2, Value)
#define SET_CCR3_VAL(Value)         WRITE_REG(TIM1->CCR3, Value)
#define SET_CCR4_VAL(Value)         WRITE_REG(TIM1->CCR4, Value)
#define SET_CCR5_VAL(Value)         WRITE_REG(TIM1->CCR5, Value)

#define DISABLE_PWMOUT()            CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE)
#define ENABLE_PWMOUT()             SET_BIT(TIM1->BDTR, TIM_BDTR_MOE)

#define READ_TIM1_UPDATE_FLAG()     READ_BIT(TIM1->SR, TIM_IT_Update)
#define CLEAN_TIM1_UPDATE_FLAG()    CLEAR_BIT(TIM1->SR, TIM_IT_Update)

#define TIM1_BREAK_ENABLE()         SET_BIT(TIM1->BDTR,TIM_BDTR_BKE)
#define TIM1_BREAK_DISABLE()        CLEAR_BIT(TIM1->BDTR,TIM_BDTR_BKE)
#define READ_TIM1_BREAK_FLAG()      READ_BIT(TIM1->SR, TIM_IT_Break)
#define CLEAN_TIM1_BREAK_FLAG()     CLEAR_BIT(TIM1->SR, TIM_IT_Break)

extern void Drv_Pwm_Init(TIM_TypeDef * pTim, uint16_t u16Period,uint16_t u16DeadTime);


/**
  * @}
*/

/**
  * @}
*/

#endif
