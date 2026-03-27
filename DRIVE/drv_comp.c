/** 
 * @file     drv_comp.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the driver functions for the COMP.
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
#define _DRV_COMP_C_

/** Files includes */
#include "drv_comp.h"

/** 
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/** 
 * @addtogroup Drv_COMP
 * @{
 */

/**
 * @brief    : This function describes the underlying configuration of the comparator.
 * @param      sSelection : Comp number
 *             pCompInput : Comp struct,include NonInvertingInput/InvertingInput/CrvSelect
 * @retval   : None
 */
void Drv_Comp_Init(COMP_Input_TypeDef * pCompInput)
{
    COMP_InitTypeDef COMP_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_COMP_Msk, ENABLE);

    COMP_DeInit(COMP1);
    COMP_InitStructure.NonInvert = pCompInput->sCompNonInvertingInput;
    COMP_InitStructure.Invert    = pCompInput->sCompInvertingInput;
    COMP_InitStructure.Output            = COMP_Output_TIM1BKIN;
    COMP_InitStructure.OutputPol         = COMP_Pol_NonInvertedOut;
    COMP_InitStructure.Hysteresis        = COMP_Hysteresis_Medium;
    COMP_InitStructure.Mode              = COMP_Mode_HighPower;
    COMP_InitStructure.OFLT            = COMP_Filter_64_Period; 
  
  
    COMP_Init(COMP1, &COMP_InitStructure);    
    
    if (COMP_InvertingInput_IO3 == pCompInput->sCompInvertingInput)
    {
      COMP_SetCrv(COMP1,COMP_CRV_Src_VDDA,pCompInput->u8CompCrvSelect);
    }
    COMP_Cmd(COMP1, ENABLE);
}

/**
  * @}
*/

/**
  * @}
*/
