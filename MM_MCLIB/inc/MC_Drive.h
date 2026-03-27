
#ifndef __MC_DRIVE_H
#define __MC_DRIVE_H

/*-------------------- Includes -----------------------*/
#include "mm32_device.h"
#include "hal_conf.h"


typedef struct
{
	u8  PWMIn;           	//PWM中断标志
	u8  Delay5ms;         //定时5ms标志
	u8  Delay200ms; 		//定时200ms标志
	u8  Delay1000ms;    //定时1s标志    (added by PZ 20230315)
}
TIMFlagType;

extern TIMFlagType TIMFlag;

extern uint8_t RunState;
extern int16_t ElecAngle;
extern int16_t IdRef;
extern int16_t IqRef;

extern void Motor_Drive(void);
extern void Motor_Model(u8 state);


#endif
