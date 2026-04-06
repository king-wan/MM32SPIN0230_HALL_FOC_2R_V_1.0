/**
 * @file     mm32_it.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides the ITR functions and test samples.
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
#define _MM32_IT_C_

/** Files includes */
#include "drv_inc.h"
#include "MC_Drive.h"

/**
 * @addtogroup MM32_User_Layer
 * @{
 */

/**
 * @addtogroup User_Main
 * @{
 */

void NVIC_Configure(uint8_t ch, uint8_t pri)
{
    NVIC_InitTypeDef  NVIC_InitStruct;

    /** Initialization ADC interrupt */
    NVIC_InitStruct.NVIC_IRQChannel = ch;
    NVIC_InitStruct.NVIC_IRQChannelPriority = pri;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void Interrupt_Init(void)
{
    /** Initialization Systick interrupt */
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_INTERRUPT);
	
    /** Initialization ADC interrupt */
    NVIC_Configure(ADC_IRQn, ADC1_INTERRUPT);
	
    /** ADC EOF interrupt enabled */
    ADC_ITConfig(ADC1, ADC_IT_EOS, ENABLE);
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOS);
	
    /** Initialization TIM interrupt */
    NVIC_Configure(TIM1_BRK_UP_TRG_COM_IRQn, TIM1_UPDATE_INTERRUPT);
	
    /** TIM Update interrupt enable */
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
	
    /** TIM Break interrupt disabled */
    TIM_ClearFlag(TIM1, TIM_FLAG_Break);
    TIM_ITConfig(TIM1, TIM_FLAG_Break, DISABLE);
}


void SysTick_Handler(void)
{
    Clear_Over_Flag();
    Inc_Systicks();
    /* todo*/
}

void WWDG_IRQHandler(void) {}

void PVD_IRQHandler(void) {}

void PWM_IRQHandler(void) {}

void FLASH_IRQHandler(void) {}

void RCC_IRQHandler(void) {}

void EXTI0_1_IRQHandler(void) {}

void EXTI2_3_IRQHandler(void) {}

void EXTI4_15_IRQHandler(void) {}

void HWDIV_IRQHandler(void) {}

void DMA1_Channel1_IRQHandler(void)
{
    if (SET == DMA_GetITStatus(USART_DMA_TX_IT_TC))
    {
        DMA_Cmd(USART_DMA_TX_CHANNEL, DISABLE);
        USART_TX_DMA_InterruptFlag = 1;
        DMA_ClearITPendingBit(USART_DMA_TX_IT_TC);
    }
}

void DMA1_Channel2_IRQHandler(void)
{
    if (SET == DMA_GetITStatus(USART_DMA_RX_IT_TC))
    {
        DMA_Cmd(USART_DMA_RX_CHANNEL, DISABLE);
        USART_RX_DMA_InterruptFlag = 1;
        DMA_ClearITPendingBit(USART_DMA_RX_IT_TC);
    }
}

void DMA1_Channel2_3_IRQHandler(void) {}

void DMA1_Channel4_5_IRQHandler(void) {}

void ADC_IRQHandler(void) 
{
	if(READ_ADC_EOC_FLAG())
    {
		CLEAN_ADC_EOC_FLAG();
		
		LED1_TOGGLE();
		
		Motor_Drive();
    }
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    if(READ_TIM1_UPDATE_FLAG())
    {
		/* clear ADC flag,very important!*/
//		ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
        CLEAN_TIM1_UPDATE_FLAG();   
    }

    if(READ_TIM1_BREAK_FLAG())
    {
        CLEAN_TIM1_BREAK_FLAG();
        /* todo*/
    }
}


void TIM1_CC_IRQHandler(void) {}

void TIM2_IRQHandler(void) {}

void TIM3_IRQHandler(void) {}

void TIM14_IRQHandler(void) {}

void TIM16_IRQHandler(void) {}

void TIM17_IRQHandler(void) {}

void I2C1_IRQHandler(void) {}

void SPI1_IRQHandler(void) {}

void SPI2_IRQHandler(void) {}

void UART1_IRQHandler(void) {}

void UART2_IRQHandler(void) {}

void UART3_IRQHandler(void) {}
    
void FLEX_CAN_IRQHandler(void) {}
/**
  * @}
*/

/**
  * @}
*/
