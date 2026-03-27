/*-------------------- Includes -----------------------*/
#include "drv_inc.h"
#include "PID.h"
#include "HallHandle.h"
#include "MC_Drive.h"
#include "FOC_Math.h"
#include "pwm_gen.h"
#include "user_function.h"
#include "Diagnose.h"
#include "parameter.h"

/*------------------- Private variables ---------------*/
TIMFlagType TIMFlag;

uint8_t MotorState = IDLESTATE;
uint16_t Time5msCnt = 0;
uint16_t Time200msCnt = 0;
int16_t ElecAngle = 0;
int16_t IdRef = -1000;
int16_t IqRef = 0;
int16_t AngleTest = 0;
Trig_Components Vector_Components;

uint32_t u32IuSum = 0;
uint32_t u32IvSum = 0;
uint16_t u16IuOffset = 0;
uint16_t u16IvOffset = 0;

/*------------------ Private functions ----------------*/
void Motor_Drive(void);
void Motor_Model(u8 state);
void MotorIdle(void);
void MotorBrake(void);
void MotorRun(void);
void MotorError(void);

/****************************************************************
	函数名：ADC1_IRQHandler
	描述：ADC转换完成中断
	输入：无
	输出：无
****************************************************************/
void Motor_Drive(void)		
{
	static uint16_t u16Cnt = 0;
	
	//时间片 1.6us
	TIMFlag.PWMIn = 1;
	
	Time5msCnt++;
	if(Time5msCnt >= 80)				//5ms
	{
		Time5msCnt = 0;
		TIMFlag.Delay5ms = 1;
	}
	
	Time200msCnt++;
	if(Time200msCnt >= 3200)			//200ms
	{
		Time200msCnt = 0;
		TIMFlag.Delay200ms = 1;
	}
	
	/* Counter overflow down*/
	if(u16Cnt <= 127)
	{
		u16Cnt++;
		u32IuSum += (int16_t)GET_ADC_VALUE(IR_U_CHANNEL);		//adc is 12 bit
		u32IvSum += (int16_t)GET_ADC_VALUE(IR_V_CHANNEL);		//adc is 12 bit
	}
	else if(u16Cnt == 128)
	{
		u16Cnt++;
		u16IuOffset = u32IuSum>>4;			//15bit offset
		u16IvOffset = u32IvSum>>4;			//15bit offset
		u32IuSum = 0;
		u32IvSum = 0;
	}
	else
	{
		ADC_Structure.IU = u16IuOffset - (GET_ADC_VALUE(IR_U_CHANNEL)<<3);	//adc is 15 bit
		ADC_Structure.IV = u16IvOffset - (GET_ADC_VALUE(IR_V_CHANNEL)<<3);	//adc is 15 bit
		ADC_Structure.VBusInput = GET_ADC_VALUE(VBUS_CHANNEL);		//adc is 12 bit
		//ADC_Structure.SPEED = GET_ADC_VALUE(VR_CHANNEL);  			//adc is 12 bit
		ADC_Structure.SPEED = 4095;  			//adc is 12 bit
	}	
	
	//逻辑转换
	Motor_Model(MotorState);
	
	//霍尔角度及故障检测
	HALLModuleCalc(&HALL1);
	HALLCheck(&HALL1);
	Vector_Components = MCM_Trig_Functions(HALL1.Angle);
	
//	AngleTest += 25;
//	Vector_Components = MCM_Trig_Functions(AngleTest);
	
	//clark
	clarke1.As = ADC_Structure.IU;
	clarke1.Bs = ADC_Structure.IV;
	CLARKE_MACRO1(&clarke1);
	
	//park
	park1.Alpha = clarke1.Alpha;
	park1.Beta = clarke1.Beta;
	park1.Sin = Vector_Components.hSin;
	park1.Cos = Vector_Components.hCos;
	PARK_MACRO1(&park1);
	
	if(FAULT.Byte == 0)
	{
		//Id Iq PI
		CurID.qInRef = IdRef;
		CurID.qInMeas = park1.Ds;
		CalcPI(&CurID);

		CurIQ.qInRef = Speed.qOut;
		CurIQ.qInMeas = park1.Qs;
		CalcPI(&CurIQ);
		
		//IPARK变换 4.5us
		ipark1.Ds = CurID.qOut;
		ipark1.Qs = CurIQ.qOut;
		ipark1.Sin = Vector_Components.hSin;
		ipark1.Cos = Vector_Components.hCos;
		IPARK_MACRO1(&ipark1);
		
		//SVPWM算法 7.4us
		pwm_gen.Alpha = ipark1.Alpha;
		pwm_gen.Beta	= ipark1.Beta;
		PWM_GEN_calc(&pwm_gen);
		
		Update_PWM(&pwm_gen);
	}
	else
	{
		MotorState = ERRORSTATE;
	}
	
	
}

/****************************************************************
	函数名：Motor_Model
	描述：控制逻辑转换
	输入：无
	输出：无
****************************************************************/
void Motor_Model(u8 state)
{
	switch(state)
	{     
		case IDLESTATE:     		MotorIdle();          	break; 
		case BRAKESTATE:			MotorBrake();			break;	
		case RUNSTATE:   			MotorRun();      		break;
		case ERRORSTATE:   			MotorError();      		break;
	}
}

/****************************************************************
	函数名：MotorIdle
	描述：电机空闲模式
	输入：无
	输出：无
****************************************************************/
void MotorIdle(void)
{
	if(RP.OutEn)  				//ADC调节速度有效后控制电机
	{
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
		MotorState = RUNSTATE;
	}
	else	
	{
		//关驱动
		TIM_CtrlPWMOutputs(TIM1, DISABLE);
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
		TIM1->CCR1FALL = 0;
		TIM1->CCR2FALL = 0;
		TIM1->CCR3FALL = 0;
		
		//清零控制变量
		CurIQ.qdSum = 0;
		CurIQ.qOut = 0;
		CurID.qdSum = 0;
		CurID.qOut = 0;
		Speed.qdSum = 0;
		Speed.qOut = 0;
		
		MotorState = IDLESTATE;
	}
}

/****************************************************************
	函数名：MotorBrake
	描述：电机空闲模式
	输入：无
	输出：无
****************************************************************/
void MotorBrake(void)
{
	if(HALL1.SpeedTemp >= 100)
	{
		//关驱动
		TIM_CtrlPWMOutputs(TIM1, DISABLE);
		
		MotorState = BRAKESTATE;
	}
	else
	{
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
		HALL1.CMDDIR = -HALL1.CMDDIR;
		MotorState = RUNSTATE;
		
		//清零控制变量
		CurIQ.qdSum = 0;
		CurIQ.qOut = 0;
		CurID.qdSum = 0;
		CurID.qOut = 0;
		Speed.qdSum = 0;
		Speed.qOut = 0;
	}
}

/****************************************************************
	函数名：MotorCloseLoop
	描述：电机定位
	输入：无
	输出：无
****************************************************************/
void MotorRun(void)
{
	if(RP.OutEn)
	{
		MotorState = RUNSTATE;
	}	
	else
	{
		MotorState = IDLESTATE;
	}
}

/****************************************************************
	函数名：MotorIdle
	描述：电机空闲模式
	输入：无
	输出：无
****************************************************************/
void MotorError(void)
{
	if(RP.OutEn)  				//ADC调节速度有效后控制电机
	{
		//关驱动
		TIM_CtrlPWMOutputs(TIM1, DISABLE);
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
		TIM1->CCR1FALL = 0;
		TIM1->CCR2FALL = 0;
		TIM1->CCR3FALL = 0;
		
		//清零控制变量
		CurIQ.qdSum = 0;
		CurIQ.qOut = 0;
		CurID.qdSum = 0;
		CurID.qOut = 0;
		Speed.qdSum = 0;
		Speed.qOut = 0;
		
		MotorState = ERRORSTATE;
	}
	else	
	{
		FAULT.Byte = 0;
		MotorState = IDLESTATE;
	}
}
