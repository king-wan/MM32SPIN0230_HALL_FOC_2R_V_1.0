/**
 * @file     drv_tim.c
 * @author   Motor TEAM
 * @brief    This file provides all the driver functions for the tim.
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
#include "drv_tim.h"

/**
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/**
 * @addtogroup Drv_TIM
 * @{
 */

/**
 * @brief    : Set TIM1 as Hall sensor function.
 * @param    : None
 * @retval   : None
 */
void Drv_Hall_Init(u32 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef		TIM_TimeBaseStruct;
	TIM_ICInitTypeDef			TIM_ICInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2_Msk, ENABLE);       //Open TIM2 clock

    TIM_DeInit(TIM2);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
    TIM_TimeBaseStruct.TIM_Period = arr;
    TIM_TimeBaseStruct.TIM_Prescaler = psc;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
	
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStruct.TIM_ICFilter = 0x0A;
    TIM_ICInit(TIM2, &TIM_ICInitStruct);    //Configure PWM

    TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable); 
	TIM_InternalClockConfig(TIM2);					//000：关闭从模式 - 如果 CEN = 1，则预分频器直接由内部时钟驱动。
    TIM_SelectHallSensor(TIM2,ENABLE);				//TIM2_CH1、 TIM2_CH2 和 TIM2_CH3 管脚经异或后作为 TI1 输入
    
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI1F_ED);   
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
    TIM_UpdateRequestConfig(TIM2,TIM_UpdateSource_Regular);
	
    /** Allow interrupts to be triggered    */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	
    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
	
	TIM_Cmd(TIM2, ENABLE);
}
