/** 
 * @file     drv_pwm.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the driver functions for the PWM.
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
#define _DRV_PWM_C_

/** Files includes */
#include "drv_pwm.h"

/**
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/**
 * @addtogroup Drv_PWM
 * @{
 */

void Drv_Pwm_Init(TIM_TypeDef * pTim, uint16_t u16Period,uint16_t u16DeadTime)
{
    /** Define the struct of the PWM configuration */
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStruct;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /** Enable the TIM1 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM1_Msk, ENABLE);

    /**
     * Sets the value of the automatic reload register Period for the next update event load activity  
     * Set the Prescaler value used as the divisor of the TIMx clock frequency
     * Set clock split :TDTS = TIM_CKD_DIV1
     * TIM center aligned mode1  
     */
    TIM_TimeBaseStructure.TIM_Period        = u16Period;
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_CenterAligned2;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;
    
    TIM_TimeBaseInit(pTim, &TIM_TimeBaseStructure);

    /**
     * Enable state selection in running mode  
     * Enable state selection in idle mode  
     * Software error lock configuration: lock closed without protection
     * DTG[7:0] dead zone generator configuration (dead zone time DT)  
     */
     /**
     * TDTS = 125nS(8MHz)
     * DTG[7: 5] = 0xx => DT = DTG[7: 0] * Tdtg, Tdtg = TDTS;
     * DTG[7: 5] = 10x => DT =(64+DTG[5: 0]) * Tdtg, Tdtg = 2 * TDTS;
     * DTG[7: 5] = 110 => DT =(32+DTG[4: 0]) * Tdtg, Tdtg = 8 * TDTS;
     * DTG[7: 5] = 111=> DT =(32 + DTG[4: 0]) *  Tdtg, Tdtg = 16 * TDTS;
     */
    TIM_BDTRInitStruct.TIM_OSSRState    = TIM_OSSRState_Enable;
    TIM_BDTRInitStruct.TIM_OSSIState    = TIM_OSSIState_Enable;
    TIM_BDTRInitStruct.TIM_LOCKLevel    = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStruct.TIM_DeadTime     = u16DeadTime;

    /**
     * Brake configuration: enable brake
     * Brake input polarity: active in low level   
     * Auto output enable configuration: Disable MOE bit hardware control
     */
    TIM_BDTRInitStruct.TIM_Break            = TIM_Break_Enable;
    TIM_BDTRInitStruct.TIM_BreakPolarity    = TIM_BreakPolarity_High;
    TIM_BDTRInitStruct.TIM_AutomaticOutput  = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(pTim, &TIM_BDTRInitStruct);

    /**
     * Mode configuration: PWM mode 2  
     * Output status setting: enable output  
     * Complementary channel output status setting: enable output
     * Sets the pulse value to be loaded into the capture comparison register
     * Output polarity is high
     * N Output polarity is high
     */

    TIM_OCInitStructure.TIM_OCMode          = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState     = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState    = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse           = 0;
    TIM_OCInitStructure.TIM_OCPolarity      = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity     = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState     = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState    = TIM_OCNIdleState_Reset;
    /*Initializes the specified parameters in the init_struct*/
    TIM_OC1Init(pTim, &TIM_OCInitStructure);
    TIM_OC2Init(pTim, &TIM_OCInitStructure);
    TIM_OC3Init(pTim, &TIM_OCInitStructure);
    
    /** Initialize the CCR4 trigger point */
    TIM_OCInitStructure.TIM_Pulse           = u16Period - 1;
    TIM_OCInitStructure.TIM_OutputState     = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_OutputNState    = TIM_OutputNState_Disable;
    
    TIM_OC4Init(pTim, &TIM_OCInitStructure);

    /** Enable CH1, 2, and 3 to be preloaded */
    TIM_OC1PreloadConfig(pTim, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(pTim, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(pTim, TIM_OCPreload_Enable);
    
    /** Enable CH4 and CH5 to be preloaded for single shunt sampling trigger*/  
    TIM_OC4PreloadConfig(pTim,TIM_OCPreload_Enable);

    /** Enable TIMx's preloaded register on ARR */
    TIM_ARRPreloadConfig(pTim, ENABLE);

    /** Enable the TIM1 */
    TIM_Cmd(pTim, ENABLE);
    /** Main Output Enable:Disable the MOE bit */
    TIM_CtrlPWMOutputs(pTim, DISABLE);
}


/**
  * @}
*/

/**
  * @}
*/
