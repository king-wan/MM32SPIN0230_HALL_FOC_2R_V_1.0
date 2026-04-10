/**
 * @file     user_function.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the functions for the board level support package.
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
#define _USER_FUNCTION_C_

/** Files includes */
#include "user_function.h"
#include "parameter.h"
#include "pwm_gen.h"
#include "HallHandle.h"
#include "PID.h"
#include "FOC_Math.h"

/*------------------- Private variables ---------------*/
int32_t IsumGain;
int32_t IshuntGain;
int32_t SpeedGain;
uint32_t VbusGain;
ADC_components ADC_Structure;

/*------------------ Private functions ----------------*/

/****************************************************************
	函数名：Init_Parameter
	描述：上电初始化变量						
	输入：无
	输出：无
****************************************************************/
void Init_Parameter(void)
{
	IshuntGain = ISHUNT_GAIN/10;								//unit 0.1A					
	IsumGain = 100*4096/ISUM_GAIN;								//unit 0.01A
	VbusGain = (122*(VBUS_PULL_UP_R + VBUS_PULL_DOWN_R)*1024)/(1000 * VBUS_PULL_DOWN_R); //unit 0.01V
	SpeedGain = 6000000/POLEPAIRS;								//RPM
	
	InitPI();
	InitNormalization(450,3950,MOTOR_COMMAND_MAX_RPM10,&RP);
	LoopCmp_Init();
	HALLModuleInit(&HALL1);
	PWM_GEN_init(&pwm_gen);
}



