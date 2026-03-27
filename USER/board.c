/**
 * @file     board.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides all the functions for the board level support package.
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
#define _BOARD_C_

/** Files includes */
#include "board.h"
#include "drv_inc.h"
#include "parameter.h"
#include "user_function.h"

/**
 * @addtogroup MM32_Hardware_Driver_Layer
 * @{
 */

/**
 * @addtogroup Bsp
 * @{
 */

/**
  * @brief OPAM  GPIO Initialization 
  * @param None
  * @retval None
  */
void Bsp_Op_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    /* GPIO Ports Clock Enable */
    RCC_AHBPeriphClockCmd(OPAMP_GPIO_CLK, ENABLE);

    	  /*Configure GPIO pin : OPAMP1_Pin */
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;

    GPIO_InitStructure.GPIO_Pin     = OPAMP1_INM_PIN;
    GPIO_Init(OPAMP1_INM_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = OPAMP1_INP_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;

    GPIO_Init(OPAMP1_INP_PORT, &GPIO_InitStructure);
  
	  /*Configure GPIO pin : OPAMP2_Pin */
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;

    GPIO_InitStructure.GPIO_Pin     = OPAMP2_INM_PIN;
    GPIO_Init(OPAMP2_INM_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = OPAMP2_INP_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;

    GPIO_Init(OPAMP2_INP_PORT, &GPIO_InitStructure);
  


}
/**
  * @brief ADC GPIO Initialization 
  * @param None
  * @retval None
  */

void Bsp_Adc_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
	
    /* ADC Clock Enable */
    RCC_AHBPeriphClockCmd(ADC_GPIO_CLK, ENABLE);
    /*Configure ADC pin  */
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;

    GPIO_InitStructure.GPIO_Pin     = VR_PIN;
    GPIO_Init(VR_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = VBUS_PIN;
    GPIO_Init(VBUS_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = IR_V_PIN;
    GPIO_Init(IR_V_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = IR_U_PIN;
    GPIO_Init(IR_U_PORT, &GPIO_InitStructure);
}

void Bsp_Comp_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
	
    /* COMP Clock Enable */
    RCC_AHBPeriphClockCmd((COMP_GPIO_CLK), ENABLE);

    GPIO_InitStructure.GPIO_Pin = COMP_INP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_High;
    GPIO_Init(COMP_INP_PORT, &GPIO_InitStructure);
}

/**
  * @brief LED GPIO Initialization 
  * @param None
  * @retval None
  */

void Bsp_Led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	
    GPIO_StructInit(&GPIO_InitStructure);
	
	  /* LED Clock Enable */
    RCC_AHBPeriphClockCmd(LED_RCC_CLOCKGPIO, ENABLE);
	
    /*Configure GPIO pin : LED_Pin */
    GPIO_InitStructure.GPIO_Pin     =  LED1_PIN;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_High;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
    GPIO_Init(LED1_PORT, &GPIO_InitStructure);
}

/**
  * @brief HALL GPIO Initialization 
  * @param None
  * @retval None
  */

void Bsp_Hall_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	
    GPIO_StructInit(&GPIO_InitStructure);
	
	/* HALL Clock Enable */
    RCC_AHBPeriphClockCmd(HALL_RCC_CLOCKGPIO, ENABLE);
	
    /*Configure GPIO pin : HALL_Pin */
    GPIO_InitStructure.GPIO_Pin     =  HALL_U_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IPU;
    GPIO_Init(HALL_U_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin     =  HALL_V_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IPU;
    GPIO_Init(HALL_V_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin     =  HALL_W_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IPU;
    GPIO_Init(HALL_W_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(HALL_U_PORT, HALL_U_PIN_SOURCE, HALL_U_PIN_AF); 	//TIM2_CH1
	GPIO_PinAFConfig(HALL_V_PORT, HALL_V_PIN_SOURCE, HALL_V_PIN_AF); 	//TIM2_CH2
	GPIO_PinAFConfig(HALL_W_PORT, HALL_W_PIN_SOURCE, HALL_W_PIN_AF); 	//TIM2_CH3
}

/**
  * @brief PWM GPIO Initialization 
  * @param None
  * @retval None
  */

void Bsp_Pwm_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    /* PWM Clock Enable */
    RCC_AHBPeriphClockCmd(BLDC1_GPIO_CLK, ENABLE);
    /*Configure GPIO pin : PWM_Pin */
    GPIO_InitStructure.GPIO_Pin     = BLDC1_UH_PIN;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_High;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
    GPIO_Init(BLDC1_UH_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BLDC1_VH_PIN;
    GPIO_Init(BLDC1_VH_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BLDC1_WH_PIN;
    GPIO_Init(BLDC1_WH_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BLDC1_UL_PIN;
#if PWM_L_USE_IO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#endif
    GPIO_Init(BLDC1_UL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BLDC1_VL_PIN;
    GPIO_Init(BLDC1_VL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BLDC1_WL_PIN;
    GPIO_Init(BLDC1_WL_PORT, &GPIO_InitStructure);
    /*selects the pin to used as Alternate function of PWM*/
    GPIO_PinAFConfig(BLDC1_UH_PORT, BLDC1_UH_PIN_SRC, BLDC1_UH_PIN_AF);
    GPIO_PinAFConfig(BLDC1_VH_PORT, BLDC1_VH_PIN_SRC, BLDC1_VH_PIN_AF);
    GPIO_PinAFConfig(BLDC1_WH_PORT, BLDC1_WH_PIN_SRC, BLDC1_WH_PIN_AF);

#if PWM_L_USE_TIM
    GPIO_PinAFConfig(BLDC1_UL_PORT, BLDC1_UL_PIN_SRC, BLDC1_UL_PIN_AF);
    GPIO_PinAFConfig(BLDC1_VL_PORT, BLDC1_VL_PIN_SRC, BLDC1_VL_PIN_AF);
    GPIO_PinAFConfig(BLDC1_WL_PORT, BLDC1_WL_PIN_SRC, BLDC1_WL_PIN_AF);
#endif
#if PWM_L_USE_IO
    GPIO_ResetBits(BLDC_UL_PORT, BLDC_UL_PIN);
    GPIO_ResetBits(BLDC_VL_PORT, BLDC_VL_PIN);
    GPIO_ResetBits(BLDC_WL_PORT, BLDC_WL_PIN);
#endif
}

/**
  * @brief Initialize all configured GPIO
  * @param None
  * @retval None
  */
void Bsp_Gpio_Init(void)
{
    Bsp_Op_Init();
    Bsp_Led_Init();
	Bsp_Hall_Init();
    Bsp_Adc_Init();
    Bsp_Comp_Init();
    Bsp_Pwm_Init();
}
/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
void Board_ADC_Init(void)
{
    /*ADC  RANK Array*/
    ADC_Channel_TypeDef sUserAdc1Channel[4];

    /* Configure the ADC RANK Sequence*/
    sUserAdc1Channel[0].u8Rank = IR_U_RANK;
    sUserAdc1Channel[0].sAdcChannel = IR_U_CHANNEL;
    sUserAdc1Channel[0].pNext = &sUserAdc1Channel[1];
	
    sUserAdc1Channel[1].u8Rank = IR_V_RANK;
    sUserAdc1Channel[1].sAdcChannel = IR_V_CHANNEL;
    sUserAdc1Channel[1].pNext = &sUserAdc1Channel[2];
	
    sUserAdc1Channel[2].u8Rank = VBUS_RANK;
    sUserAdc1Channel[2].sAdcChannel = VBUS_CHANNEL;
    sUserAdc1Channel[2].pNext = &sUserAdc1Channel[3];
	
    sUserAdc1Channel[3].u8Rank = VR_RANK;
    sUserAdc1Channel[3].sAdcChannel = VR_CHANNEL;
    sUserAdc1Channel[3].pNext = NULL;
	
	/* Select the ADC external trigger source of the ADC is T1_CC5*/
    Drv_Adc_Basic_Init(ADC1, ADC_ExtTrig_T1_CC4);
     /* Select the ADC sample time*/
    Drv_Adc_Channel_Init(ADC1, sUserAdc1Channel, ADC_SampleTime_2_5); 

    ADC_Cmd(ADC1, ENABLE);
}

volatile COMP_TypeDef *pComp;

void Board_Comp_Init(void)
{
    COMP_Input_TypeDef sUserCompInput;
	  /* Select the inverting input of the comparator */
    sUserCompInput.sCompInvertingInput = COMP_INVERTING;
	  /* Select the non inverting input of the comparator*/
    sUserCompInput.sCompNonInvertingInput = COMP_NON_INVERTING;
	  /* Select comparator external reference voltage */
    sUserCompInput.u8CompCrvSelect = COMP_CRV_VOLTAGE_SELECT;
	  /* Initializes the COMP according to the specified parameters in the COMP_Input_TypeDef */
    Drv_Comp_Init(&sUserCompInput);
}
/**
* @brief    : This function describes the underlying configuration of the op-amp.
* @param    : None
* @retval   : None
*/
void Board_Opamp_Init(void)
{
	 /* op-amp Clock Enable */
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_OPA1_Msk, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_OPA2_Msk, ENABLE);
    /*Enable the specified OPAMP peripheral*/
    OPAMP_Cmd(OPAMP1,ENABLE);
    OPAMP_Cmd(OPAMP2,ENABLE);
}
/**
  * @brief Initialize all configured peripherals
  * @param None
  * @retval None
  */
void Peripheral_Init(void)
{
    Board_ADC_Init();
    Board_Comp_Init();
    Board_Opamp_Init();
	Drv_Hall_Init(TIM2_PRIOD, TIM2_PSC_LOAD);
    Drv_Pwm_Init(TIM1, PWMPERIOD, DEAD_TIME);
    Drv_Iwdg_Init();
	
	/*Initialize divider*/
    Drv_Hwdiv_Init();
	
	/** Enable the TIM1 */
    TIM_Cmd(TIM1, ENABLE);
}


/**
  * @}
*/

/**
  * @}
*/
