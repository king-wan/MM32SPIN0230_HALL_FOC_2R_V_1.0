/** 
 * @file     drv_adc.h
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the driver functions prototypes for the ADC.
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
#ifndef __DRV_ADC_H
#define __DRV_ADC_H

/** Files includes */
#include <stdio.h>
#include "hal_adc.h"

/** 
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/** 
 * @addtogroup Drv_ADC
 * @{
 */
 

 
typedef struct ADC_Channel
{
    uint8_t u8Rank;
    uint8_t sAdcChannel;
    struct ADC_Channel * pNext;
}ADC_Channel_TypeDef;

 
#define READ_ADC_EOC_FLAG()      READ_BIT(ADC1->ADSTA, ADC_IT_EOC)
#define CLEAN_ADC_EOC_FLAG()     SET_BIT(ADC1->ADSTA, ADC_IT_EOC)
#define READ_ADC_JEOS_FLAG()     READ_BIT(ADC1->ADSTA_EXT, ADC_ADSTA_EXT_JEOSIF_Msk)
#define CLEAN_ADC_JEOS_FLAG()    SET_BIT(ADC1->ADSTA_EXT, (0x01U << ADC_ADSTA_EXT_JEOSIF_Pos))

/** ADC Sequential sampling */
#define GET_ADC_VALUE(Channel)      (READ_REG(*(&(ADC1->ADDR0) + Channel)) & 0xFFF)
#define GET_ADC_INJECT_VALUE(adc, channel)  (READ_REG(*(&(adc->JDR0) + channel)) & 0xFFF)

extern void Drv_Adc_Basic_Init(ADC_TypeDef* pAdc, uint32_t ADC_ExternalTrigConv);
extern void Drv_Adc_Channel_Init(ADC_TypeDef* pAdc, ADC_Channel_TypeDef* pAdcChannel,uint32_t s32SampleTime);
extern void Drv_Adc_Injected_Channel_Init(ADC_TypeDef* pAdc, ADC_Channel_TypeDef* pAdcChannel, uint32_t s32SampleTime, uint32_t u32TrigSource);

/**
  * @}
*/

/**
  * @}
*/


#endif
