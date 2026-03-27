/**
 * @file     board.h
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the functions prototypes for the board level support package.
 *
 * @attention
 *
 * THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 * CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 * TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 * HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 * CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 *
 * <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
 */

/** Define to prevent recursive inclusion */
#ifndef __BOARD_H
#define __BOARD_H

/** Files includes */
#include <stdio.h>
#include "mm32_device.h"
#include "hal_conf.h"

/** Interrupt priorities */
/**
 * The order of priority decreases from small to large
 * 0 : highest priority-first
 */
#define SYSTICK_INTERRUPT           (1)
#define TIM1_UPDATE_INTERRUPT       (2)
#define ADC1_INTERRUPT              (0)

/** ADC interface */
#define IR_U_RANK                   (0)
#define IR_U_CHANNEL                ADC_Channel_5 

#define IR_V_RANK                   (1)
#define IR_V_CHANNEL                ADC_Channel_4 

#define VBUS_RANK                   (2) 
#define VBUS_CHANNEL                ADC_Channel_1

#define VR_RANK                     (3)
#define VR_CHANNEL                  ADC_Channel_0  


#define ADC_GPIO_CLK               (RCC_AHBENR_GPIOA_Msk | RCC_AHBENR_GPIOB_Msk)

#define VR_PORT                     GPIOA
#define VR_PIN                      GPIO_Pin_1

#define VBUS_PORT                   GPIOA
#define VBUS_PIN                    GPIO_Pin_2

#define IR_U_PORT                   GPIOB
#define IR_U_PIN                    GPIO_Pin_0

#define IR_V_PORT                   GPIOA
#define IR_V_PIN                    GPIO_Pin_5

/** COMP interface */
#define COMP_NUMBER                 COMP1
#define COMP_NON_INVERTING          COMP_NonInvertingInput_IO3                                      /** for PA4 */

#define COMP_INVERTING              COMP_InvertingInput_IO3//COMP_InvertingInput_CRV
#define COMP_CRV_VOLTAGE_SELECT     30// means: (30+1)/64

#define COMP_GPIO_CLK               (RCC_AHBENR_GPIOA_Msk)

#define COMP_INP_PORT               GPIOB
#define COMP_INP_PIN                GPIO_Pin_0

/** OPAMP interface */
#define OPAMP_GPIO_CLK              (RCC_AHBENR_GPIOA_Msk | RCC_AHBENR_GPIOB_Msk)
#define OPAMP1_INM_PORT             GPIOA
#define OPAMP1_INM_PIN              GPIO_Pin_7
#define OPAMP1_INP_PORT             GPIOA
#define OPAMP1_INP_PIN              GPIO_Pin_6
#define OPAMP2_INM_PORT             GPIOA
#define OPAMP2_INM_PIN              GPIO_Pin_4
#define OPAMP2_INP_PORT             GPIOA
#define OPAMP2_INP_PIN              GPIO_Pin_3


/** LED interface */
#define LED_RCC_CLOCKGPIO           (RCC_AHBENR_GPIOB_Msk)

#define LED1_PORT                   GPIOB
#define LED1_PIN                    GPIO_Pin_9
#define LED1_PIN_SOURCE             GPIO_PinSource9

#define LED1_ON()                   LED1_PORT->BRR = LED1_PIN
#define LED1_OFF()                  LED1_PORT->BSRR = LED1_PIN
#define LED1_TOGGLE()               LED1_PORT->ODR ^= LED1_PIN

/** HALL interface */
#define HALL_RCC_CLOCKGPIO          (RCC_AHBENR_GPIOA_Msk | RCC_AHBENR_GPIOB_Msk)

#define HALL_U_PORT              	GPIOA
#define HALL_U_PIN              	GPIO_Pin_9

#define HALL_V_PORT             	GPIOA
#define HALL_V_PIN               	GPIO_Pin_8

#define HALL_W_PORT               	GPIOB
#define HALL_W_PIN                	GPIO_Pin_2

#define HALL_U_PIN_SOURCE        	GPIO_PinSource9
#define HALL_V_PIN_SOURCE         	GPIO_PinSource8
#define HALL_W_PIN_SOURCE         	GPIO_PinSource2

#define HALL_U_PIN_AF              	GPIO_AF_0
#define HALL_V_PIN_AF              	GPIO_AF_0
#define HALL_W_PIN_AF              	GPIO_AF_0

/** PWM interface */
#define PWM_L_USE_IO   0
#define PWM_L_USE_TIM  1

#define BLDC1_GPIO_CLK               (RCC_AHBENR_GPIOA_Msk | RCC_AHBENR_GPIOB_Msk)

#define BLDC1_UH_PORT                GPIOB
#define BLDC1_UH_PIN                 GPIO_Pin_7
#define BLDC1_VH_PORT                GPIOB
#define BLDC1_VH_PIN                 GPIO_Pin_5
#define BLDC1_WH_PORT                GPIOB
#define BLDC1_WH_PIN                 GPIO_Pin_3

#define BLDC1_UL_PORT                GPIOB
#define BLDC1_UL_PIN                 GPIO_Pin_6
#define BLDC1_VL_PORT                GPIOB
#define BLDC1_VL_PIN                 GPIO_Pin_4
#define BLDC1_WL_PORT                GPIOA
#define BLDC1_WL_PIN                 GPIO_Pin_15

//#define BLDC1_BKP_PORT               GPIOA
//#define BLDC1_BKP_PIN                GPIO_Pin_6

#define BLDC1_UH_PIN_SRC             GPIO_PinSource7
#define BLDC1_VH_PIN_SRC             GPIO_PinSource5
#define BLDC1_WH_PIN_SRC             GPIO_PinSource3
#define BLDC1_UL_PIN_SRC             GPIO_PinSource6
#define BLDC1_VL_PIN_SRC             GPIO_PinSource4
#define BLDC1_WL_PIN_SRC             GPIO_PinSource15
//#define BLDC1_BKP_PIN_SRC            GPIO_PinSource6

#define BLDC1_UH_PIN_AF              GPIO_AF_5
#define BLDC1_VH_PIN_AF              GPIO_AF_7
#define BLDC1_WH_PIN_AF              GPIO_AF_7
#define BLDC1_UL_PIN_AF              GPIO_AF_1
#define BLDC1_VL_PIN_AF              GPIO_AF_7
#define BLDC1_WL_PIN_AF              GPIO_AF_5
//#define BLDC1_BKP_PIN_AF             GPIO_AF_2




/**
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/**
 * @addtogroup Bsp
 * @{
 */


extern void Bsp_Gpio_Init(void);
extern void Peripheral_Init(void);
/**
  * @}
*/

/**
  * @}
*/


#endif
