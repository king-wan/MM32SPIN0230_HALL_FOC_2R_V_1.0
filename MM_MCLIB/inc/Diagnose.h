#ifndef Diagnose_h
#define Diagnose_h

#include "mm32_device.h"
#include "hal_conf.h"

//added by PZ 20230315
#define FAULT_RETRY_CNT           65535     // retry count, max 65534, 65535 means endless retry
#define FAULT_RETRY_CNT_CRITICAL  5         // retry count for critical fault (eg. hardware OC)
#define FAULT_RETRY_DELAY         5         // unit: second, delay this value after fault brought out, then restart

typedef union {
				struct
				{
					unsigned OverVBUSFlag :1;			//¹ýÑ¹
					unsigned UnderVBUSFlag :1;	 		//Ç·Ñ¹
					unsigned OverTempFlag :1;	 		//¹ýÎÂ
					unsigned OverIBUSFlag :1;			//¹ýÁ÷
					unsigned HallFlag :1;				//»ô¶û´íÎó
					unsigned LockedFlag :1;				//¶Â×ª
					unsigned LackPhaseFlag :1;	 		//È±Ïà
					unsigned BrakeFlag :1;				//Ó²¼þ¹ÊÕÏ
				}bit;
				u8 Byte;
} DiagFlag;

extern DiagFlag FAULT;

extern void  Diagnose_IBUS_ADC(u16 I_ADCData);
extern void  Diagnose_VBUS_ADC(u16 V_ADCData);
extern void  Diagnose_MotorBlock(void);
extern void  Diagnose_Lack_Phase(void);
extern void  Diagnose_Temp(u16 T_ADCData);
extern void  Diagnose_MotorBlock(void);
uint8_t Diagnose_Retry(void);
void Diagnose_Retry_Release(void);
void Diagnose_Retry_Clear(void);



#endif
