/**
 * @file     main.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides the main functions and test samples.
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
#define _MAIN_C_

/** Files includes */
#include "main.h"
#include "drv_inc.h"
#include "board.h"
#include "user_function.h"
#include "MC_Drive.h"
#include "Diagnose.h"
#include "FOC_Math.h"
#include "HallHandle.h"
#include "PID.h"

/**
 * @addtogroup MM32_User_Layer
 * @{
 */
int16_t M1FaultID, M1FaultID_Record;
/**
 * @addtogroup User_Main
 * @{
 */

int main(void)
{
	 /* Configure the system clock */
    Systick_Init(SystemCoreClock / 1000);
    Systick_Delay(200);
	
	 /* Initialize  motor control parameters */
    Init_Parameter();
	
	 /* Initialize all GPIO */
    Bsp_Gpio_Init();
	
	 /* Initialize all configured peripherals */
    Peripheral_Init();
	
	 /* Initialize interrupts */
    Interrupt_Init();
	
	 /* Infinite loop */
    while(1)
    {
		/*IWDG_ReloadCounter*/
        IWDG_RELOAD_COUNT();
		
		if(TIMFlag.PWMIn == 1)		//电流、电压诊断
		{
			TIMFlag.PWMIn = 0;
			
			//电流保护
			if(ADC_Structure.IBusInput <= 0)
			{
				ADC_Structure.IBusInput = 0;
				ADC_Structure.IBusAvrg = 0;
			}
			else
			{
				ADC_Structure.IBusInput = (IsumGain*ADC_Structure.IBusInput)>>12;
				ADC_Structure.IBusAvrg = (ADC_Structure.IBusAvrg*7 + ADC_Structure.IBusInput)>>3;
			}
			
			//过流保护
			Diagnose_IBUS_ADC(ADC_Structure.IBusAvrg);

		}
		
		if(TIMFlag.Delay5ms == 1)	//速度环调节
		{
			TIMFlag.Delay5ms = 0;
			
			//参考速度归一化
			CalcNormalization(ADC_Structure.SPEED,&RP);

			//加减速控制
			RPValue.Dest = RP.Out;
			LoopCmp_Cal(&RPValue);
			
			//速度环
			SpeedFdk.NewData = HALL1.SpeedTemp;
			MovingAvgCal(&SpeedFdk);
			
			Speed.qInRef = RPValue.Act;
			Speed.qInMeas = SpeedFdk.Out;
			CalcPI(&Speed);
			
			//========================
			// -------- Linear Field Weakening --------
			if (SpeedFdk.Out <= 3200)
			{
					IdRef = 0;
			}
			else if (SpeedFdk.Out >= 5000)
			{
					IdRef = -700;
			}
			else
			{
					// IdRef = -(Speed-3200) * 700 / (5000-3200)
					IdRef = -((int32_t)(SpeedFdk.Out - 3200) * 700 / 1800);
			}
			//========================
			
			//电压保护 缺相保护
			Diagnose_VBUS_ADC(ADC_Structure.VBusInput);
		}
    }
}

/**
  * @}
*/

/**
  * @}
*/
