/** 
 * @file     drv_div.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the driver functions for the div.
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
#define _DRV_DIV_C_

/** Files includes */
#include "drv_div.h"

/**
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/**
 * @addtogroup Drv_DIV
 * @{
 */


static void HDIV_SignInit(void)
{
    HWDIV->CR &= (~HWDIV_CR_USIGN) ;
}

void Drv_Hwdiv_Init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBENR_HWDIV_Msk, ENABLE);

    HDIV_SignInit();
}

int32_t Division(int32_t m, int32_t n)
{
    if(n == 0)
    {
        return 0;
    }
    if(m == 0)
    {
        return 0;
    }

    HWDIV->DVDR = m;
    HWDIV->DVSR = n;
    return (HWDIV->QUOTR);
}

/**
  * @}
*/

/**
  * @}
*/
