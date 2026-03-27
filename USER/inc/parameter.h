#ifndef __PARAMETER_H_
#define __PARAMETER_H_

#define  SingleShunt

/*System setting*/
#define SYS_REFV             	5.0         //  unit:v      MCU VCC must be 5.0V or 3.3V  
#define SYSCLK_HSI_60MHz		60000000    //  unit:Hz

#define PWMFREQ 				16000      //  unit:Hz
#define PWMPERIOD				SYSCLK_HSI_60MHz/PWMFREQ/2				
#define DEAD_TIME               60        //determined by Hardware parameter   

#define TIM2_PSC_LOAD 			(u16)(SYSCLK_HSI_60MHz/1000000-1) 	//定时器2装载值
#define TIM2_PRIOD    			99999                          		//定时器2周期值

#endif
