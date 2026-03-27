#ifndef __USER_SUB_H
#define __USER_SUB_H

#include "mm32_device.h"
#include "hal_conf.h"

#define		POLEPAIRS		1
#define 	MAX_SPEED		4000
#define 	MOTOR_DIR		-1	      	//1:CW, -1:CCW

//--------------------- Current measurement -------------------------------------------------------------------
#define 	ISUM_R_VALUE    		50 	//unit:milli ohm, 50 means 0.05 ohm
#define 	ISUM_AMP_FACTOR  		50 	//unit:0.1 amplification factor, 50 means 50*0.1 = 5 amplification factor
#define 	ISHUNT_R_VALUE    		100 //unit:milli ohm, 50 means 0.05 ohm
#define 	ISHUNT_AMP_FACTOR		50 	//unit:0.1 amplification factor, 50 means 50*0.1 = 5 amplification factor
#define 	ISUM_GAIN 		((ISUM_R_VALUE*ISUM_AMP_FACTOR*32768)/50000) 		//50*5*32768/5000 = 1638---1A
#define 	ISHUNT_GAIN 	((ISHUNT_R_VALUE*ISHUNT_AMP_FACTOR*32768)/50000) 	//50*5*32768/5000 = 1638---1A

//--------------------- DC Bus voltage measurement --------------------------------------------------------------
#define 	VBUS_PULL_UP_R     1000 	//unit : 0.1K Ohm, 1000 means 100K Ohm //20181215
#define 	VBUS_PULL_DOWN_R   100  	//unit : 0.1K Ohm, 100  means 10K  Ohm //20181215

#define 	VBUS_HIGH_VALUE1	3600  		//过压阀值		36V
#define 	VBUS_HIGH_VALUE2	3500  		//过压恢复		35V
#define 	VBUS_LOW_VALUE1		1600   	 	//欠压阀值		20V
#define 	VBUS_LOW_VALUE2		1700   	  	//欠压恢复		21V
#define 	IBUS_OVER_VALUE		200   		//软件过流值		2A
#define 	IBUS_LIMIT_VALUE	120			//软件限流		1A
#define 	LACK_PHASE_DETECT_CYCLE		1000

//电机状态表
#define 	IDLESTATE      	0
#define		RUNSTATE  		1
#define 	BRAKESTATE     	2
#define 	ERRORSTATE     	3

typedef struct
{ 
	u16 SPEED;
	u16	VBusInput;
	u16 VBusAvrg;
	s16	IBusInput;
	s16 IBusAvrg;
	s16 IU;
	s16 IV;
	s16 EU;
	s16 EV;
	s16 EW;
}ADC_components; 	


extern int32_t IsumGain;
extern int32_t IshuntGain;
extern int32_t SpeedGain;
extern uint32_t VbusGain;
extern ADC_components ADC_Structure;

extern void Init_Parameter(void);

#endif
