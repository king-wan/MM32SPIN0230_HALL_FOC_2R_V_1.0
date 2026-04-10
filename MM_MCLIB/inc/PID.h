#ifndef PID_h
#define PID_h

#define DUTY_OUTMAX  2249    					//占空比输出最大值
#define DUTY_OUTMIN  20             	//占空比输出最小值


typedef struct {
	   signed   long  qdSum;
	   signed   long  qdSumMax;
	   signed   long  qdSumMin;
	   signed   long  qPrevErr;
	   signed   int   qKp;
	   signed   int   qKpDiv;
	   signed   int   qKi;
	   signed   int   qKiDiv;
	   signed   int   qKd;
	   signed   int   qKdDiv;
	   signed   int   qKiPrev;
	   signed   int   qOutMaxPrev;
	   signed   int   qOutMinPrev;
	   signed   int   qOutMax;
	   signed   int   qOutMin;
	   signed   int   qInRef;
	   signed   int   qInMeas;
	   signed   int   qOut;
}tPIParm;

extern void InitPI(void);
extern void CalcPI(tPIParm *pParm);

extern tPIParm CurID;
extern tPIParm CurIQ;
extern tPIParm Speed;
extern tPIParm PLL;
extern tPIParm Position;

extern void InitPI(void); //PI初始化程序
extern void CalcPI(tPIParm *pParm); //PI运算程序

#endif
