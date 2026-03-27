/*******************************************************************************
Copyright (2020~2023),MindMotion 
FileName: Diagnose.c
Author: Hunter Zhu
Date:   2022.08.15
Description: 过流、过压、欠压故障检测
Version: v0.0

Function List: 
1.Diagnose_IBUS_ADC            			过流检测及限流调节
2.Diagnose_VBUS_ADC                		过压、欠压检测

History: 2022/8/15 v0.0  build this moudle
*******************************************************************************/

/*-------------------- Includes -----------------------*/
#include "Diagnose.h"
#include "FOC_Math.h"
#include "PID.h"
#include "MC_Drive.h"
#include "user_function.h"

/*------------------- Private variables ---------------*/
DiagFlag FAULT;				
uint16_t MotorLocked_State = 0;
uint16_t MotorOCP_State = 0;
uint16_t Counter_LackPhase = 0;
uint32_t u32IaSumForLackPhase = 0;
uint32_t u32IbSumForLackPhase = 0;
uint32_t u32IcSumForLackPhase = 0;
uint16_t u16RetryDelay = 0;
uint16_t u16RetryCNT = 0;
uint16_t u16RetryCNTCritical = 0;
uint16_t u16RetryReleaseCNT = 0;

/****************************************************************
	函数名：Diagnose_IBUS_ADC
	描述：过流保护								AD:18.6 --- 1A
	输入：无
	输出：无
****************************************************************/
void Diagnose_IBUS_ADC(u16 I_ADCData)
{
	if(I_ADCData >= IBUS_OVER_VALUE)  
	{
		MotorOCP_State++;
	}
	else
	{
		MotorOCP_State = 0;
	}
	
	if(MotorOCP_State >= 32)
	{
		//FAULT.bit.OverIBUSFlag = 1;
	}
	else
	{}
}

/****************************************************************
	函数名：Diagnose_VBUS_ADC
	描述：过压、欠压保护
	输入：无
	输出：无
****************************************************************/
void Diagnose_VBUS_ADC(u16 V_ADCData)
{
	static u32 VBUS_High_State;
	static u32 VBUS_Low_State;
	
	ADC_Structure.VBusAvrg = ((uint32_t)V_ADCData * VbusGain)>>10;		//unit : 10mV 
	
	//过压
	if(ADC_Structure.VBusAvrg > VBUS_HIGH_VALUE1)
	{
		VBUS_High_State <<= 1;
		VBUS_High_State |= 0x0001;
	}
	else 
	{
		if(ADC_Structure.VBusAvrg < VBUS_HIGH_VALUE2)
		{
			VBUS_High_State <<= 1;
			VBUS_High_State &= 0xfffe;
		}
	}
	
	//欠压
	if(ADC_Structure.VBusAvrg < VBUS_LOW_VALUE1)
	{
		VBUS_Low_State <<= 1; 
		VBUS_Low_State |= 0x0001;
	}
	else 
	{
		if(ADC_Structure.VBusAvrg > VBUS_LOW_VALUE2)
		{
			VBUS_Low_State <<= 1; 
			VBUS_Low_State &= 0xfffe;
		}
	}
	
	if((VBUS_High_State&0xFFFF) == 0xFFFF)
	{
		FAULT.bit.OverVBUSFlag = 1;
	}
	else 
	{
		if(VBUS_High_State == 0)
		{
			FAULT.bit.OverVBUSFlag = 0;	
		}
	}
	
	if((VBUS_Low_State&0xFFFF) == 0xFFFF)
	{
		FAULT.bit.UnderVBUSFlag = 1;
	}
	else 
	{
		if(VBUS_Low_State == 0)
		{
			FAULT.bit.UnderVBUSFlag = 0;
		}
	}
}


/****************************************************************
	函数名：Diagnose_MotorLocked
	描述：堵转保护							
	输入：无
	输出：无										
****************************************************************/
void  Diagnose_MotorBlock(void)
{
//	MotorLocked_State <<= 1;
//	
//	//闭环堵转||开环堵转
//	if(((PLLData.Out >= (PLL.qOutMax-20))&&(CurIQ.qOut < 6000))
//		||((RunState == OPENLOOP)&&(BlockTime == 200)))
//	{
//		MotorLocked_State |= 0x01;
//	}
//	else
//	{
//		MotorLocked_State &= 0xfe;
//	}
//	
//	if(MotorLocked_State >= 0x1F)  // 1S
//	{
//		FAULT.bit.LockedFlag = 1;
//	}
//	else if(MotorLocked_State == 0)
//	{
//		FAULT.bit.LockedFlag  = 0;
//	}
//	else
//	{}
}

/****************************************************************
	函数名：Diagnose_Temp
	描述：过温保护
	输入：无
	输出：无
****************************************************************/
void Diagnose_Temp(u16 T_ADCData)
{
	//电机温度保护
	if(T_ADCData <=  439)		//90度以上 0.354V
	{
		FAULT.bit.OverTempFlag = 1;
	}
	else if(T_ADCData <= 738)	//70~90度	
	{
	}
	else			//低于70度	0.595V
	{
		FAULT.bit.OverTempFlag = 0;
	}
}

/********************************************************************************************************
**function name        ：Diagnose_Lack_Phase            
**function description ：detect the motor 3 phase lines connection is ok or not, if not, it will stop motor  
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Diagnose_Lack_Phase(void)  
{	
	uint8_t  TempLackPhaseErrorFlag = 0;	
	
	//detect lack phase error at 3 mode, RUN_IN_OPEN_LOOP_MODE,RUN_IN_CLOSE_LOOP_MODE,RUN_IN_NEW_STARTUP_MODE
//	if((MotorState == RUNSTATE)&&(SpeedFdk.Out >= 30))
//	{
//		Counter_LackPhase++;

//		if(clarke1.As < 0)
//		{
//			u32IaSumForLackPhase += (-clarke1.As);
//		}
//		else
//		{
//			u32IaSumForLackPhase += (clarke1.As);
//		} //calculation Ia sum

//		if(clarke1.Bs < 0)
//		{
//			u32IbSumForLackPhase += (-clarke1.Bs);
//		}
//		else
//		{
//			u32IbSumForLackPhase += (clarke1.Bs);
//		} //calculation Ib sum

//		if(clarke1.Cs < 0)
//		{
//			u32IcSumForLackPhase += (-clarke1.Cs);
//		}
//		else
//		{
//			u32IcSumForLackPhase += (clarke1.Cs);
//		} //calculation Ic sum		
//	}
//	else
//	{
//		Counter_LackPhase = 0;//clear the counter
//		u32IaSumForLackPhase = 0;//clear to 0
//		u32IbSumForLackPhase = 0;//clear to 0
//		u32IcSumForLackPhase = 0;//clear to 0
//	}
	
	if(Counter_LackPhase >= LACK_PHASE_DETECT_CYCLE)
	{
		Counter_LackPhase = 0;			//clear the counter
		TempLackPhaseErrorFlag = 0;
		
		if(u32IaSumForLackPhase > u32IbSumForLackPhase)
		{ 
			if(u32IaSumForLackPhase > (4*u32IbSumForLackPhase))
			{
				TempLackPhaseErrorFlag = 1;
			}
		}		
		else
		{ 
			if(u32IbSumForLackPhase > (4*u32IaSumForLackPhase))
			{
				TempLackPhaseErrorFlag = 1;
			}
		}		
		
		if(u32IaSumForLackPhase > u32IcSumForLackPhase)
		{ 
			if(u32IaSumForLackPhase > (4*u32IcSumForLackPhase))
			{
				TempLackPhaseErrorFlag = 1;
			}
		}		
		else
		{ 
			if(u32IcSumForLackPhase > (4*u32IaSumForLackPhase))	
			{
				TempLackPhaseErrorFlag = 1;
			}
	  }		
		
		
		if(u32IbSumForLackPhase > u32IcSumForLackPhase)
		{ 
			if(u32IbSumForLackPhase > (4*u32IcSumForLackPhase))
			{
				TempLackPhaseErrorFlag = 1;
			}
		}		
		else
		{ 
			if(u32IcSumForLackPhase > (4*u32IbSumForLackPhase))
			{
				TempLackPhaseErrorFlag = 1;
			}
		}		

		
		if(TempLackPhaseErrorFlag ==1)
		{
			FAULT.bit.LackPhaseFlag = 1;	 
		}
		
		u32IaSumForLackPhase = 0;//clear to 0
		u32IbSumForLackPhase = 0;//clear to 0
		u32IcSumForLackPhase = 0;//clear to 0
	}
}	
/********************************************************************************************************
**function name        ：Diagnose_Retry
**function description ：Retry to fix the fault. Called in MotorError() every 1 second.
**                       Retry count and delay time according to FAULT_RETRY_CNT and FAULT_RETRY_DELAY
**                       separately. Refer to Diagnose.h for details.
**input parameters     ：None
**output parameters    ：Return 1 means delay time has reach, need to restart.
********************************************************************************************************/
uint8_t Diagnose_Retry(void)
{
	uint8_t res = 0;
		
	if(FAULT.bit.BrakeFlag != 1)   // non-critical fault
	{
		if(u16RetryCNT < FAULT_RETRY_CNT)
		{
			if(++u16RetryDelay > FAULT_RETRY_DELAY)
			{
				u16RetryDelay = 0;
				u16RetryReleaseCNT = 0;
				
			#if FAULT_RETRY_CNT < 65535
				u16RetryCNT += 1;
			#endif
				
				res = 1;
			}
		}
	}
	else   // critical fault
	{
		if(u16RetryCNTCritical < FAULT_RETRY_CNT_CRITICAL)
		{
			if(++u16RetryDelay > FAULT_RETRY_DELAY)
			{
				u16RetryDelay = 0;
				u16RetryReleaseCNT = 0;				
				u16RetryCNTCritical += 1;				
				res = 1;
			}
		}
	}
	
	return res;
}

/********************************************************************************************************
**function name        ：Diagnose_Retry_Release (added by PZ 20230315)
**function description ：Fault has been recovered, reset u16RetryCNT. Called in MotorCloseLoop() every 1 second.
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Diagnose_Retry_Release(void)
{
	if(u16RetryCNT != 0)
	{
		if(++u16RetryReleaseCNT >= 10)  // motor could run 10 second means fault has clear, reset retry count.
		{
			u16RetryCNT = 0;
			u16RetryReleaseCNT = 0;
		}
	}
}

/********************************************************************************************************
**function name        ：Diagnose_Retry_Clear (added by PZ 20230315)
**function description ：VSP signal disappeared, stop retry action and clear all the variables using in
**                     : retry module. Called in  MotorError() when VSP signal disappeares. 
**input parameters     ：None
**output parameters    ：None
********************************************************************************************************/
void Diagnose_Retry_Clear(void)
{
	u16RetryCNT = 0;
	u16RetryReleaseCNT = 0;
	u16RetryDelay = 0;
}
