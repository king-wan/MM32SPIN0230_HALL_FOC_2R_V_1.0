/*
 *******************************************************************************
    @file     hal_uart.c
    @file     hal_uart.c
    @author   VV TEAM
    @brief    THIS FILE PROVIDES ALL THE USART FIRMWARE FUNCTIONS.
 *******************************************************************************
    @attention

    <h2><center>&copy; Copyright(c) <2023> <MindMotion></center></h2>

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the
    following conditions are met:
    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its
       contributors may be used to endorse or promote products derived from this
       software without specific prior written permission.

    THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
    CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
    TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
    CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
    HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
    CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.

    <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
 *******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#define _HAL_USART_C_

/* Files includes ------------------------------------------------------------*/
#include "hal_usart.h"

/** @addtogroup MM32_Hardware_Abstract_Layer
  * @{
  */

/** @addtogroup USART_HAL
  * @{
  */

/** @addtogroup USART_Exported_Functions
  * @{
  */

/**
* @brief  Deinitializes the USARTx peripheral registers to their
*   default reset values.
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1.
* @retval : None
*/
void USART_DeInit(USART_TypeDef *usart)
{
    if(USART1 == usart) 
    {
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART1, ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART1, DISABLE);
    }
}

/**
* @brief  Initializes the USARTx peripheral according to the specified
*   parameters in the USART_InitStruct .
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1
* @param init_struct: pointer to a USART_InitTypeDef structure
*   that contains the configuration information for the
*   specified USART peripheral.
* @retval : None
*/
void USART_Init(USART_TypeDef *usart, USART_InitTypeDef *init_struct)
{
    uint32_t tmpreg1 = 0x00, tmpreg2 = 0x00, tmpreg3 = 0x00, apbclock = 0x00;

    uint32_t USARTxbase = 0;
    RCC_ClocksTypeDef RCC_ClocksStatus;
    USARTxbase = (*(uint32_t*)&usart);

    /* get USART CR1,2,3 values */
    tmpreg1 = usart->CR1;
    tmpreg2 = usart->CR2;
    tmpreg3 = usart->CR3;
    /* Clear valid bits */
    tmpreg1 &= ~(USART_CR1_RE_Msk | USART_CR1_TE_Msk | USART_CR1_PS_Msk | USART_CR1_DL_Msk);
    tmpreg2 &= ~USART_CR2_STOP_Msk;
    tmpreg3 &= ~(USART_CR3_RTSE_Msk | USART_CR3_CTSE_Msk);
    /* Configure the USART Bits,*/
    tmpreg1 |= (uint32_t)init_struct->USART_WordLength | init_struct->USART_Parity | init_struct->USART_Mode;
    tmpreg2 |= (uint32_t)init_struct->USART_StopBits;
    tmpreg3 |= (uint32_t)init_struct->USART_HardwareFlowControl;
    /* Write to USART CR1,2,3 */
    usart->CR1 = tmpreg1;
    usart->CR2 = tmpreg2;
    usart->CR3 = tmpreg3;
    /*---------------------------- USART BRR Configuration -----------------------*/
    /* Configure the USART Baud Rate -------------------------------------------*/
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    if (USARTxbase == USART1_BASE)
    {
        apbclock = RCC_ClocksStatus.PCLK1_Frequency;
    }
	/* Write to USART BRR */
	usart->BRR &= (~0x000fffff);
	usart->BRR |= (((apbclock/ 16) /init_struct->USART_BaudRate) << 4);//DIV
	usart->BRR |= (((apbclock/(init_struct->USART_BaudRate))%16) << 0);//FRA
}

/**
* @brief  USART_StructInit.
* @param init_struct: pointer to a USART_InitTypeDef structure
*   that contains the configuration information for the
*   specified USART peripheral.
* @retval : None
*/
void USART_StructInit(USART_InitTypeDef *init_struct)
{
    /* USART_InitStruct members default value */
    init_struct->USART_BaudRate = 9600;
    init_struct->USART_WordLength = USART_WordLength_8b;
    init_struct->USART_StopBits = USART_StopBits_1;
    init_struct->USART_Parity = USART_Parity_No ;
    init_struct->USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    init_struct->USART_HardwareFlowControl = USART_HardwareFlowControl_None;
}

/**
* @brief  Enables or disables the specified USART peripheral.
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1.
* @param state: new state of the USARTx peripheral.
*   This parameter can be: ENABLE or DISABLE.
* @retval : None
*/
void USART_Cmd(USART_TypeDef *usart, FunctionalState state)
{
    (state) ? \
    (usart->CR1 |=  (0x01U << USART_CR1_UE_Pos)): \
    (usart->CR1 &= ~(0x01U << USART_CR1_UE_Pos));
}

/**
* @brief  Enables or disables the specified USART CR1 interrupts.
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1, USART2, USART3.
* @param usart_it: specifies the USART interrupt sources to be
*   enabled or disabled.
*   This parameter can be one of the following values:
* @arg   USART_IT_IDLEIEN
* @arg   USART_IT_RXIEN  
* @arg   USART_IT_TCIEN  
* @arg   USART_IT_TXIEN  
* @arg   USART_IT_PEIEN  
* @param NewState: new state of the specified USARTx interrupts.
*   This parameter can be: ENABLE or DISABLE.
* @retval : None
*/
void USART_ITConfig(USART_TypeDef *usart, uint16_t usart_it, FunctionalState state)
{
    (state) ? \
    (usart->CR1 |=  usart_it): \
    (usart->CR1 &= ~usart_it);
}

/**
* @brief  Enables or disables the USART DMA interface.
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1.
* @param NewState: new state of the DMA Request sources.
*   This parameter can be: ENABLE or DISABLE.
* @note The DMA mode is not available for USART5.
* @retval : None
*/
void USART_DMACmd(USART_TypeDef *usart, FunctionalState state)
{
    (state) ? \
    (usart->CR3 |=  (0x01U << USART_CR3_DMAMODE_Pos)): \
    (usart->CR3 &= ~(0x01U << USART_CR3_DMAMODE_Pos));

}

/**
* @brief  Transmits single data through the USARTx peripheral.
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1.
* @param data: the data to transmit.
* @retval : None
*/
void USART_SendData(USART_TypeDef *usart, uint16_t data)
{
    /* Transmit Data */
	usart->DR = data & 0x1FFU;
}

/**
* @brief  Returns the most recent received data by the USARTx peripheral.
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1.
* @retval : The received data.
*/
uint16_t USART_ReceiveData(USART_TypeDef *usart)
{
    /* Receive Data */
    return (uint16_t)(usart->DR & (uint16_t)0x01FF);
}

/**
* @brief  Checks whether the specified USART flag is set or not.
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1.
* @param usart_flag: specifies the flag to check.
*   This parameter can be one of the following values:
* @arg   USART_FLAG_PE   
* @arg   USART_FLAG_FE	
* @arg   USART_FLAG_NF	
* @arg   USART_FLAG_ORE	
* @arg   USART_FLAG_IDLE	
* @arg   USART_FLAG_RXNE 
* @arg   USART_FLAG_TC   
* @arg   USART_FLAG_TXE  
* @arg   USART_FLAG_CTS	
* @retval : The new state of USART_FLAG (SET or RE
*/
FlagStatus USART_GetFlagStatus(USART_TypeDef *usart, uint16_t usart_flag)
{
    FlagStatus bitstatus = RESET;
    if ((usart->SR & usart_flag) != (uint16_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }
    return bitstatus;
}

/**
* @brief  Clears the USARTx's pending flags.
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1.
* @param usart_flag: specifies the flag to clear.
*   This parameter can be any combination of the following values:
* @arg   USART_FLAG_RXNE 
* @arg   USART_FLAG_CTS	  
* @retval : None
*/
void USART_ClearFlag(USART_TypeDef *usart, uint16_t usart_flag)
{
    usart->SR &= (~(usart_flag));
}

/**
* @brief  Checks whether the specified USART CR1 interrupt has occurred or not.
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1.
* @param usart_it: specifies the flag to check.
*   This parameter can be any combination of the following values:
* @arg  USART_IT_PE     
* @arg  USART_IT_FE		
* @arg  USART_IT_NF		
* @arg  USART_IT_ORE    
* @arg  USART_IT_IDLE	
* @arg  USART_IT_RXNE   
* @arg  USART_IT_TC     
* @arg  USART_IT_TXE    
* @arg  USART_IT_CTS	
* @retval : The new state of USART_IT (SET or RESET).
*/
ITStatus USART_GetITStatus(USART_TypeDef *usart, uint16_t usart_it)
{
    FlagStatus bitstatus = RESET;
    if ((usart->SR & usart_it) != (uint16_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }
    return bitstatus;
}

/**
* @brief  Clears the USARTx interrupt pending bits.
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1.
* @param usart_it: specifies the interrupt pending bit to clear.
*   This parameter can be one of the following values:
* @arg  USART_IT_PE     
* @arg  USART_IT_FE		
* @arg  USART_IT_NF		
* @arg  USART_IT_ORE    
* @arg  USART_IT_IDLE	
* @arg  USART_IT_RXNE   
* @arg  USART_IT_TC         
* @arg  USART_IT_CTS	
* @retval : None
*/
void USART_ClearITPendingBit(USART_TypeDef *usart, uint16_t usart_it)
{
    /*clear USART_IT pendings bit*/
    usart->SR = usart_it;
}

/**
* @brief  Initializes the SYNC MASTER USARTx peripheral according to the specified
*   parameters in the USART_InitStruct .
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1, USART2, USART3.
* @param mode: Set MODE.
*   This parameter can be one of the following values:
*   USART_SYNC_MODE0, SART_SYNC_MODE1, USART_SYNC_MODE2, USART_SYNC_MODE3
* @param usart_bound: speed.
* @retval : None
*/
void USART_SyncConfigure(USART_TypeDef *usart, uint8_t mode, uint32_t usart_bound)
{
    uint32_t apbclock = 0x00;

    uint32_t USARTxbase = 0;
    RCC_ClocksTypeDef RCC_ClocksStatus;

    USARTxbase = (*(uint32_t*)&usart);

    /*---------------------------- USART BRR Configuration -----------------------*/
    /* Configure the USART Baud Rate -------------------------------------------*/
    RCC_GetClocksFreq(&RCC_ClocksStatus);
    if (USARTxbase == USART1_BASE)
    {
        apbclock = RCC_ClocksStatus.PCLK1_Frequency;
    }
    apbclock = 8000000;//220906 add
    /* Write to USART BRR */
    usart->BRR &= (~0x000fffff);
    usart->BRR |= (((apbclock/ 4) /usart_bound) << 4);//DIV

    usart->CR1 |= (USART_CR1_MLS_Msk | USART_CR1_SAS_Msk);
    usart->CR3 &= ~USART_CR3_HDSEL_Msk;

    MODIFY_REG(usart->CR2, USART_CR2_CPHA_Msk | USART_CR2_CPOL_Msk, mode); 

    usart->CR1 |= (USART_CR1_RE_Msk | USART_CR1_TE_Msk);
}

/**
* @brief  Initializes the SYNC SLAVE USARTx peripheral according to the specified
*   parameters in the USART_InitStruct .
* @param usart: Select the USART or the USART peripheral.
*   This parameter can be one of the following values:
*   USART1
* @param mode: Set MODE.
*   This parameter can be one of the following values:
*   USART_SYNC_MODE0, SART_SYNC_MODE1, USART_SYNC_MODE2, USART_SYNC_MODE3
* @retval : None
*/
void USART_SyncSlaveConfigure(USART_TypeDef *usart, uint8_t mode)
{
    usart->CR1 |= (USART_CR1_MLS_Msk | USART_CR1_SAS_Msk);
    usart->CR3 |= USART_CR3_CKINE_Msk;
    usart->CR3 &= ~USART_CR3_HDSEL_Msk;

    MODIFY_REG(usart->CR2, USART_CR2_CPHA_Msk | USART_CR2_CPOL_Msk, mode); 

    usart->CR1 |= (USART_CR1_RE_Msk | USART_CR1_TE_Msk);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
