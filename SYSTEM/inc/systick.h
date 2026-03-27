/** 
 * @file     systick.h
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides config and functions for the systick.
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
#ifndef __SYSTICK_H
#define __SYSTICK_H 			   

/** Files includes */
#include "HAL_conf.h"
#include "mm32_device.h"

/** 
 * @addtogroup MM32_System_Layer
 * @{
 */

/** 
 * @addtogroup Systick
 * @{
 */

extern void Systick_Init(uint32_t ticks);
extern uint32_t Get_Systick_Cnt(void);
extern uint32_t Get_Systick_Val(void);
extern uint32_t Get_Over_Flag(void);
extern void Clear_Over_Flag(void);
extern void Inc_Systicks(void);

extern void Systick_Delay(volatile uint32_t Delay);

/**
  * @}
*/

/**
  * @}
*/

#endif

