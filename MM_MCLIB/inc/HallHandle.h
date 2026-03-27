
#ifndef __HALLHANDLE_H
#define __HALLHANDLE_H

#include "mm32_device.h"
#include "hal_conf.h"


typedef struct
{
	uint8_t RunHallValue;   	
	uint8_t PreHallValue;   
	int8_t	CMDDIR;
	int16_t	Angle;
	int16_t IncAngle;
	int16_t IncAngleMax;
	int16_t SpeedTemp;
	uint8_t Time100msCNT;
	uint32_t HallTime[8];		
	uint32_t HallTimeSum;
	int16_t CWAngleTab[8];
	int16_t CCWAngleTab[8];
}
HALLType;

typedef struct
{
  uint8_t StudyFlag;
	int16_t HallAngle;
	int16_t HallTab[6];
	int16_t	StudyAngle[8];
	int8_t HallIndex;
	uint16_t ForceTime;
	int8_t StudyCNT;
}
SectorType;

extern HALLType HALL1;					
extern SectorType SectorStudy;
extern uint8_t ReadHallValue;
extern int16_t PLL_AngleShift;

extern void HALLModuleInit(HALLType *u);
extern void HALLModuleCalc(HALLType *u);
extern void SectorModuleInit(SectorType *u);
extern void SectorModuleCalc(SectorType *u);
extern uint8_t HALL_ReadHallPorts(void);
extern void HALLCheck(HALLType *u);
extern void HallAnglePLL(void);

#endif
