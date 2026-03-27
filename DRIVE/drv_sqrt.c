/** 
 * @file     drv_sqrt.c
 * @author   Motor TEAM
 * @brief    This file provides all the driver functions for the SQRT.
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
#define _DRV_SQRT_C_

/** Files includes */
#include "drv_sqrt.h"

/**
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/**
 * @addtogroup Drv_SQRT
 * @{
 */



void Drv_Sqrt_Init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBENR_HWSQRT, ENABLE);
}

int16_t Hw_Sqrt(int32_t m)
{
    if(m == 0)
    {
        return 0;
    }

    SQRT->SQR = m;
    return (SQRT->RESULT);
}
/**
  * @}
*/

/**
  * @}
*/
