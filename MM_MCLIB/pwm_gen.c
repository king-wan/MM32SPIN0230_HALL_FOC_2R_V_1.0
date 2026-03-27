/*-------------------- Includes -----------------------*/
#include "drv_inc.h"
#include "pwm_gen.h"
#include "FOC_Math.h"
#include "parameter.h"

/*------------------- Private variables ---------------*/
PWM_GEN_T pwm_gen;
	
/*------------------ Private functions ----------------*/
void PWM_GEN_init(PWM_GEN_T *u1);
void PWM_GEN_calc(PWM_GEN_T *u1);
void Update_PWM(PWM_GEN_T *u1);

/*************************************
	函数名：PWM_GEN_init
	描述：初始化SVPWM结构体
	输入：*u1---PWM_GEN_T结构体变量
	输出：无
**************************************/
void PWM_GEN_init(PWM_GEN_T *u1)
{
	u1->Alpha = 0;
	u1->Beta	= 0;
	u1->CompA	= 0;
	u1->CompB	= 0;
	u1->CompC	= 0;
	u1->Mode	= SEVENMODE;
	u1->Sector	= 0;
	u1->N_halfPeriod	= PWMPERIOD;
}

/*************************************
	函数名：PWM_GEN_calc
	描述：计算七段式和五段式占空比
	输入：*u1---PWM_GEN_T结构体变量
	输出：无
**************************************/
void PWM_GEN_calc(PWM_GEN_T *u1)
{
	s32	tempA = 0;
	s32 tempB = 0;
	
	s32 Vref1 = 0;
	s32 Vref2 = 0;
	s32 Vref3 = 0;
	u8 sector = 0;

	s32 t1on = 0;
	s32 t2on = 0;
	s32 taon = 0;
	s32 tbon = 0;
	s32 tcon = 0;
	
	//计算三相极性
	tempA = (28378*(s32)u1->Alpha)>>15;
	tempB = (u1->Beta)>>1;
	
	Vref1 = u1->Beta;
	Vref2 = tempA - tempB;		//Valpha*sqrt(3)/2 - Vbeta/2 
	Vref3 = -tempA - tempB;		//-Valpha*sqrt(3)/2 - Vbeta/2   
	
	//扇区计算
	sector = 0;
	if(Vref1>=0)
	{
		sector = sector + 1;
	}
	if(Vref2>=0)
	{
		sector = sector + 2;
	}
	if(Vref3>=0)
	{
		sector = sector + 4;
	}
	
	//计算两个基本矢量的持续时间t1、t2，死区处理暂无
	switch(sector)
	{
		case 1:  //扇区2
		t1on =  -Vref3;              
		t2on =  -Vref2;
		break;

		case 2: //扇区6
		t1on = -Vref1;              
		t2on = -Vref3;
		break;

		case 3: //扇区1
		t1on = Vref1;               
		t2on = Vref2;     
		break;

		case 4: //扇区4         
		t1on = -Vref2;              
		t2on = -Vref1;
		break;

		case 5: //扇区3
		t1on = Vref3;               
		t2on = Vref1;
		break;

		case 6: //扇区5
		t1on = Vref2;               
		t2on = Vref3;
		break;	

		default:
		break;			
		
	}

	//计算切换时间
	taon = (32767-t1on-t2on)>>1;		//(1-t1-t2)/2,Q15格式
	tbon = taon + t1on;					//taon+t1
	tcon = tbon + t2on;					//tbon+t2
		
	//换算成占空比
	taon = (taon * u1->N_halfPeriod)>>15;

	if(taon >= u1->N_halfPeriod)
	{
		taon = u1->N_halfPeriod - 1;
	}
	else if(taon<=0)
	{
		taon = 0;
	}
	else
	{ }
		
	tbon = (tbon * u1->N_halfPeriod)>>15;

	if(tbon >= u1->N_halfPeriod)
	{
		tbon = u1->N_halfPeriod - 1;
	}
	else if(tbon<=0)
	{
		tbon = 0;
	}
	else
	{ }	

	tcon = (tcon * u1->N_halfPeriod)>>15;

	if(tcon >= u1->N_halfPeriod)
	{
		tcon = u1->N_halfPeriod - 1;
	}
	else if(tcon<=0)
	{
		tcon = 0;
	}
	else
	{ }

		
	//分发到三相矢量切换时间点Ta、Tb、Tc
	if(u1->Mode == SEVENMODE)
	{
		switch(sector)	//七段式调制
		{
			case 1:       //扇区2
			u1->CompA = tbon;
			u1->CompB = tcon;      
			u1->CompC = taon;
			break;

			case 2:       //扇区6
			u1->CompA = tcon;     
			u1->CompB = taon;
			u1->CompC = tbon;
			break;

			case 3:       //扇区1
			u1->CompA = tcon;    
			u1->CompB = tbon;    
			u1->CompC = taon;
			break;

			case 4:       //扇区4
			u1->CompA = taon;  
			u1->CompB = tbon;
			u1->CompC = tcon;
			break;

			case 5:       //扇区3
			u1->CompA = taon;   
			u1->CompB = tcon;
			u1->CompC = tbon;
			break;

			case 6:       //扇区5
			u1->CompA = tbon;
			u1->CompB = taon;   
			u1->CompC = tcon;
			break;

			default: 
			u1->CompA = 0;
			u1->CompB = 0;
			u1->CompC = 0;	
			break;
		}
	}
	else if(u1->Mode == FIVEMODE)
	{
		switch(sector)	//五段式调制
		{
				case 1:       //扇区2
				u1->CompA = tbon - taon;
				u1->CompB = tcon - taon;      
				u1->CompC = 0;
				break;

				case 2:       //扇区6
				u1->CompA = tcon - taon;     
				u1->CompB = 0;
				u1->CompC = tbon - taon;
				break;

				case 3:       //扇区1
				u1->CompA = tcon - taon;    
				u1->CompB = tbon - taon;    
				u1->CompC = 0;
				break;

				case 4:       //扇区4
				u1->CompA = 0;  
				u1->CompB = tbon - taon;
				u1->CompC = tcon - taon;
				break;

				case 5:       //扇区3
				u1->CompA = 0;   
				u1->CompB = tcon - taon;
				u1->CompC = tbon - taon;
				break;

				case 6:       //扇区5
				u1->CompA = tbon - taon;
				u1->CompB = 0;   
				u1->CompC = tcon - taon;
				break;

				default: 
				u1->CompA = 0;
				u1->CompB = 0;
				u1->CompC = 0;	
				break;
		}
	}
	else
	{
		u1->CompA = 0;
		u1->CompB = 0;
		u1->CompC = 0;
	}
	
	u1->Sector = sector;
}

/****************************************************************
	函数名：Update_PWM
	描述：更新占空比
	输入: *u1 --- PWM_GEN_T结构体变量
	输出：无
****************************************************************/
void Update_PWM(PWM_GEN_T *u1)
{
	TIM1->CCR1 = u1->CompA;
	TIM1->CCR2 = u1->CompB;
	TIM1->CCR3 = u1->CompC;
}
