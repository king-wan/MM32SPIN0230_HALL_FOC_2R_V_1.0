/*******************************************************************************
*
* Copyright 
* 
*
****************************************************************************//*!
*
* @brief  mlib_types.h
* 
*******************************************************************************/
#ifndef _MLIB_TYPES_H_
#define _MLIB_TYPES_H_

/******************************************************************************
* Includes
******************************************************************************/
#include <stdint.h>                        /* stdint.h include for data types */

/*******************************************************************************
* Types
*******************************************************************************/
/* Boolean data types */
typedef unsigned short bool_t;

/* Fractional data types */
typedef signed char             Q7_t; 
typedef signed short            Q15_t;           
typedef signed long             Q31_t;           
typedef signed long long        Q63_t;           

/* floating data types */ 

typedef float 								float32_t;

typedef double 								float64_t;

/* trigonometric structer for Angle */ 
typedef struct
{
  Q15_t s16Cos;
  Q15_t s16Sin;
} sAngle_Trig;

/*******************************************************************************
* Macros
*******************************************************************************/  

/* bool */
#ifndef FALSE
#define FALSE    ((bool_t)0)                       
#endif

#ifndef TRUE
#define TRUE     ((bool_t)1)                        
#endif

/* unsigned min, max */
#ifndef INT16_MIN
#define INT16_MIN               ((int16_t) 0x8000)
#endif

#ifndef INT16_MAX
#define INT16_MAX               ((int16_t) 0x7fff)
#endif

#ifndef INT32_MIN
#define INT32_MIN               ((int32_t) 0x80000000)
#endif

#ifndef INT32_MAX
#define INT32_MAX               ((int32_t) 0x7fffffff)
#endif

#ifndef INT64_MIN
#define INT64_MIN               ((int32_t) 0x8000000000000000)
#endif

#ifndef INT64_MAX
#define INT64_MAX               ((int32_t) 0x7fffffffffffffff)
#endif

/* unsigned min, max */
#ifndef UINT16_MAX
#define UINT16_MAX              ((uint16_t) 0x8000U)
#endif

#ifndef UINT32_MAX
#define UINT32_MAX              ((uint32_t) 0x80000000U)
#endif


/* Fractional conversion macros */
#if !defined(Q7)
#define Q7(x) ((Q7_t)((x) < 0.9921875 ? ((x) >= -1 ? (x)*0x80 : 0x80) : 0x7F))
#endif //Q7

#if !defined(Q15)
#define Q15(x) ((Q15_t)((x) < 0.999969482421875 ? ((x) >= -1 ? (x)*0x8000 : 0x8000) : 0x7FFF))
#endif //Q15

#if !defined(Q31)
#define Q31(x) ((Q31_t)((x) < 1 ? ((x) >= -1 ? (x)*0x80000000 : 0x80000000) : 0x7FFFFFFF))
#endif //Q31

#if !defined(Q63)
#define Q63(x) ((Q63_t)((x) < 1 ? ((x) >= -1 ? (x)*0x8000000000000000 : 0x8000000000000000) : 0x7FFFFFFFFFFFFFFF))
#endif //Q63

/* Other conversion macros */
#if !defined(Q8_7)
#define Q8_7(x) ((int16_t)((x) < 255.9921875 ? ((x) >= -256 ? (x)*0x80 : 0x8000) : 0x7FFF))
#endif //Q8_7

#if !defined(Q16_15)
#define Q16_15(x) ((int32_t)((x) < 65535.999969482421875 ? ((x) >= -65536 ? (x)*0x8000 : 0x80000000) : 0x7FFFFFFF))
#endif //Q16_15

#endif  /*_MLIB_TYPES_H_*/
