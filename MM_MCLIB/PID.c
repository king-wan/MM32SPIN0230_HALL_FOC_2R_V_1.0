/*-------------------- Includes -----------------------*/
#include "PID.h"
#include "MC_Drive.h"
#include "user_function.h"

/*------------------- Private variables ---------------*/
tPIParm CurID;		//ID环
tPIParm CurIQ;		//IQ环
tPIParm Speed;		//速度环
tPIParm PLL;		//锁相环

/****************************************************************
	函数名：InitPI
	描述：位置式PI初始化
	输入：无
	输出：无
****************************************************************/
void InitPI(void)
{
	CurID.qKp = 9000;				//Q15
	CurID.qKi = 200;
	CurID.qOutMax = 4000;
	CurID.qOutMin = -4000;
	CurID.qdSum = 0;
	CurID.qInMeas = 0;
	CurID.qOut = 0;
	
	CurIQ.qKp = 9000;
	CurIQ.qKi = 200;
	CurIQ.qOutMax = 30000;
	CurIQ.qOutMin = -30000;
	CurIQ.qdSum = 0;
	CurIQ.qInMeas = 0;
	CurIQ.qOut = 0;
	
	Speed.qKp = 6000;
	Speed.qKi = 80;
	Speed.qOutMax = IshuntGain * 5;
	Speed.qOutMin = -IshuntGain * 5;
	Speed.qdSum = 0;
	Speed.qInMeas = 0;
	Speed.qOut = 0;
	
	PLL.qKp = 2000;		//1<<15;
	PLL.qKi = 10;		//1800;
	PLL.qOutMax = 500;
	PLL.qOutMin = -500;
	PLL.qdSum = 0;
	PLL.qInRef = 0;
	PLL.qInMeas = 0;
	PLL.qOut = 0;
}

/****************************************************************
	函数名：CalcPI1
	描述：位置式PI计算
	输入：pParm---位置式PI结构体
	输出：无
****************************************************************/
void CalcPI(tPIParm *pParm)
{
	signed int   Error;
	signed long  Up;
	signed long  Ui;
	
	//Calc Error
	Error = pParm->qInRef - pParm->qInMeas;
	
	//Calc proportional
	Up = (pParm-> qKp * Error)>>15;
	
	//Calc integral
	pParm->qdSum = pParm->qdSum + pParm->qKi* Error;
	
	//Limit integral 
	if(pParm->qdSum >= (pParm->qOutMax<<15))		//CurIQ.qOutMax<<15
	{
		pParm->qdSum = (pParm->qOutMax<<15);
	}
	else if(pParm->qdSum <= (pParm->qOutMin<<15))
	{
		pParm->qdSum = (pParm->qOutMin<<15);
	}
	else
	{}
	
	Ui = pParm->qdSum>>15;
	
	//Out = Up + Ui 
	pParm->qOut = Up + Ui;
		
	//Out Limit
	if(pParm->qOut > pParm->qOutMax)
	{
		pParm->qOut = pParm->qOutMax ;
	}
	else if(pParm->qOut < pParm->qOutMin)
	{
		pParm->qOut = pParm->qOutMin;
	}
	else
	{
		
	}
}

