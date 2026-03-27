#ifndef FOC_Math_h
#define FOC_Math_h

#include "mm32_device.h"
#include "hal_conf.h"

#define INV_SQRT3		591				// (1/sqrt(3)) << 10

typedef struct
{
	int16_t hCos;
	int16_t hSin;
} Trig_Components;


typedef struct 
{  
	int32_t  As;  		// Input: phase-a stator variable
	int32_t  Bs;		// Input: phase-b stator variable
	int32_t  Cs;		// Input: phase-c stator variable  
	int16_t  Alpha;		// Output: stationary d-axis stator variable 
	int16_t  Beta;		// Output: stationary q-axis stator variable
}CLARKE_T;	 

typedef struct 
{  
	int32_t  Alpha;		// Output: stationary d-axis stator variable
	int32_t  Beta;		// Output: stationary q-axis stator variable
	int32_t  Theta;		// Input: rotating angle (pu)
	int32_t  Ds;		// Input: rotating d-axis stator variable
	int32_t  Qs;		// Input: rotating q-axis stator variable
	int32_t  Sin;		// Input: Sine term
	int32_t  Cos;		// Input: Cosine term 	 
}PARK_T;	            

typedef volatile struct 
{  
	int32_t  Alpha;		// Output: stationary d-axis stator variable
	int32_t  Beta;		// Output: stationary q-axis stator variable
	int16_t  Theta;		// Input: rotating angle (pu)
	int32_t  Ds;		// Input: rotating d-axis stator variable
	int32_t  Qs;		// Input: rotating q-axis stator variable
	int16_t  Sin;		// Input: Sine term
	int16_t  Cos;		// Input: Cosine term
} IPARK_T;

/*-----------------------------------------------------------------------------
	Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/                     
#define CLARKE_DEFAULTS { 0, 0, 0, 0, 0} 

/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/                     
#define PARK_DEFAULTS { 0, 0, 0, 0, 0, 0, 0}

/*-----------------------------------------------------------------------------
Default initalizer for the IPARK object.
-----------------------------------------------------------------------------*/                     
#define IPARK_DEFAULTS { 0, 0, 0, 0, 0, 0, 0}

typedef struct
{
	s16	NewData;
	u8	Index;
	s16 DataARR[32];
	s32 DataSum;
	s16	Out;
}MovingAvgData;

typedef struct
{
	s16 Dest;		//参考目标量
	s16 Act;		//实际使用量
	s16 Max;		//最大限制值
	s16 Min;		//最小限制值
	s16 Inc;		//加速步距
	s16 Dec;		//加速步距
}LoopCMP_T;

typedef struct
{
	u16 In;             //输入值
	u16 MaxIn;          //最大值有效值
	u16 MinIn;          //最小值有效值		
	u16 MaxOut;         //对应允许最大输出值
	u16 Out;            //输出值
	u8  OutEn;          //输出使能标志
	u8  State;          //输入状态
	float Kslope;       //对应斜率
}
NormalizationType; //参数归一化的相关变量

extern LoopCMP_T RPValue;
extern NormalizationType RP;
extern MovingAvgData SpeedFdk;
extern CLARKE_T clarke1;	
extern CLARKE_T Emf_AB;
extern CLARKE_T PLL_clarke;
extern PARK_T park1;
extern PARK_T PLL_park1;
extern IPARK_T ipark1;
extern Trig_Components Vector_PLL;

extern Trig_Components MCM_Trig_Functions(int16_t hAngle);
extern int32_t MCM_Sqrt(int32_t wInput);
extern int16_t arctan2(int16_t y1,int16_t x1);
extern void CLARKE_MACRO1(CLARKE_T *u1);
extern void PARK_MACRO1(PARK_T *u1);
extern void IPARK_MACRO1(volatile IPARK_T *v);
extern void MovingAvgInit(MovingAvgData *Data);
extern void MovingAvgCal(MovingAvgData *Data);
extern void LoopCmp_Init(void);
extern void LoopCmp_Cal(LoopCMP_T *u1);
extern void InitNormalization(u16 Lowdata,u16 highdata,u16 maxout,NormalizationType *u);
extern void CalcNormalization(u16 value,NormalizationType *u);

#endif
