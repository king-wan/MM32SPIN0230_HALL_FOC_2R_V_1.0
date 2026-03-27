/*-------------------- Includes -----------------------*/
#include "drv_inc.h"
#include "HallHandle.h"
#include "MC_Drive.h"
#include "user_function.h"
#include "FOC_Math.h"
#include "PID.h"
#include "Diagnose.h"

/*------------------- Private variables ---------------*/
HALLType HALL1;
SectorType SectorStudy;
int16_t CWShift = 9000;
int16_t CCWShift = -1820;
uint8_t ReadHallValue;

/*------------------ Private functions ----------------*/
void HALLModuleInit(HALLType *u);
void HALLModuleCalc(HALLType *u);
uint8_t HALL_ReadHallPorts(void);
void HALLCheck(HALLType *u);

/****************************************************************
	函数名：HALLModuleInit
	描述：霍尔模块初始化
	输入：无
	输出：无
****************************************************************/
void HALLModuleInit(HALLType *u)
{
	uint8_t i;
	
	u->RunHallValue = HALL_ReadHallPorts();
	u->PreHallValue = u->RunHallValue;
	u->CMDDIR = MOTOR_DIR;
	u->IncAngle = 5;
	u->IncAngleMax = 10922;
	u->SpeedTemp = 0;
	u->Time100msCNT = 0;
	u->HallTimeSum = 60000;
	
	u->CWAngleTab[5] = -10922;
	u->CWAngleTab[4] = 0;
	u->CWAngleTab[6] = 10922;
	u->CWAngleTab[2] = 21844;
	u->CWAngleTab[3] = 32767;
	u->CWAngleTab[1] = -21844;
	
	u->CCWAngleTab[5] = -21844;
	u->CCWAngleTab[4] = -10922;
	u->CCWAngleTab[6] = 0;
	u->CCWAngleTab[2] = 10922;
	u->CCWAngleTab[3] = 21844;
	u->CCWAngleTab[1] = 32767;
	
	
	if(u->CMDDIR == -1)
	{
		u->Angle = u->CCWAngleTab[HALL1.RunHallValue] + 5460;
	}
	else
	{
		u->Angle = u->CWAngleTab[HALL1.RunHallValue] + 5460;
	}
	
	for(i=0;i<8;i++)
	{
		u->HallTime[i] = 10000;
	}
}

/****************************************************************
	函数名：HALLModuleCalc
	描述：霍尔插值补偿
	输入：HALL--霍尔结构体
	输出：无
****************************************************************/
void HALLModuleCalc(HALLType *u)
{
	static uint8_t i = 0;
	uint8_t j = 0;
	
	u->RunHallValue = HALL_ReadHallPorts();
	
	if(u->PreHallValue != u->RunHallValue)
	{
		u->Time100msCNT = 0;
		
		i++;
		if(i >= 6)
		{
			i = 0;
		}
		
		//霍尔捕获时间
		u->HallTime[i] = TIM2->CCR1;	//采集周期
				
		//转速计算
		u->HallTimeSum = 0;
		for(j=0;j<6;j++)
		{
			u->HallTimeSum += u->HallTime[j];
		}
		
		u->SpeedTemp = Division(SpeedGain,u->HallTimeSum);
		
		//计算角度增量 62.5us
		u->IncAngle = Division(4096000,u->HallTimeSum);
		u->IncAngleMax = 10922;
		
		if(u->CMDDIR == -1)
		{
			u->Angle = u->CCWAngleTab[u->RunHallValue] + CCWShift;
		}
		else
		{
			u->Angle = u->CWAngleTab[u->RunHallValue] + CWShift;
		}
	}
	
	u->PreHallValue = u->RunHallValue;
	
	//增量值限制
	if((u->IncAngleMax - u->IncAngle)>=0)
	{
		u->IncAngleMax = u->IncAngleMax - u->IncAngle;
		u->Angle = u->Angle + u->CMDDIR*HALL1.IncAngle;
	}
	else
	{ }
}

/****************************************************************
	函数名：HALL_ReadHallPorts
	描述：检测霍尔值
	输入：无
	输出：无
****************************************************************/
uint8_t HALL_ReadHallPorts(void)
{
	uint8_t HallA;
	uint8_t HallB;
	uint8_t HallC;
	uint8_t HallValue;
	
	HallA = GPIO_ReadInputDataBit(HALL_U_PORT,HALL_U_PIN);
	HallB = GPIO_ReadInputDataBit(HALL_V_PORT,HALL_V_PIN);
	HallC = GPIO_ReadInputDataBit(HALL_W_PORT,HALL_W_PIN);
	HallValue = HallA * 1 + HallB * 2 + HallC * 4; 
	
	return HallValue;
}

/****************************************************************
	函数名：HALLModuleCalc
	描述：霍尔插值补偿
	输入：*u--霍尔结构体
	输出：无
****************************************************************/
void HALLCheck(HALLType *u)
{
	if((u->RunHallValue == 0)||(u->RunHallValue == 7))	
	{
		FAULT.bit.HallFlag = 1;
	}
}

