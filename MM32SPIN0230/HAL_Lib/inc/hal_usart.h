/*
 *******************************************************************************
    @file     hal_usart.h
    @author   VV TEAM
    @brief    THIS FILE CONTAINS ALL THE FUNCTIONS PROTOTYPES FOR THE USART
              FIRMWARE LIBRARY.
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
#ifndef __HAL_USART_H
#define __HAL_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Files includes ------------------------------------------------------------*/
#include "mm32_device.h"

/** @addtogroup MM32_Hardware_Abstract_Layer
  * @{
  */

/**
* @brief  USART Init Structure definition
*/

typedef struct
{
    uint32_t USART_BaudRate;
    uint16_t USART_WordLength;
    uint16_t USART_StopBits;
    uint16_t USART_Parity;
    uint16_t USART_Mode;
    uint16_t USART_HardwareFlowControl;
} USART_InitTypeDef;

/**
* @}
*/

/** @defgroup USART_Exported_Constants
* @{
*/

/** @defgroup USART_Word_Length
* @{
*/
#define USART_WordLength_8b                  (0x00U << USART_CR1_DL_Pos)
#define USART_WordLength_9b                  (0x01U << USART_CR1_DL_Pos)

/**
* @}
*/

/** @defgroup USART_Stop_Bits
* @{
*/
#define USART_StopBits_1                     (0x00U << USART_CR2_STOP_Pos)      /*!< 1 STOP bit */
#define USART_StopBits_2                     (0x02U << USART_CR2_STOP_Pos)      /*!< 2 STOP bit */

/**
* @}
*/

/** @defgroup USART_Parity
* @{
*/
#define USART_Parity_No                      (0x00U << USART_CR1_PS_Pos)  
#define USART_Parity_Even                    (0x00U << USART_CR1_PS_Pos)  
#define USART_Parity_Odd                     (0x01U << USART_CR1_PS_Pos)   

/**
* @}
*/

/** @defgroup USART_Mode
* @{
*/
#define USART_Mode_Rx                 (0x01U << USART_CR1_RE_Pos)             /*!< Receiver enable */
#define USART_Mode_Tx                 (0x01U << USART_CR1_TE_Pos)             /*!< Transmitter enable */

/**
* @}
*/

/** @defgroup USART_EN
* @{
*/
#define USART_EN                      (0x01U << USART_CR1_UE_Pos)             /*!< USART enable */

/**
* @}
*/

/** @defgroup USART_Hardware_Flow_Control
* @{
*/
#define USART_HardwareFlowControl_None       	((0x00U << USART_CR3_RTSE_Pos) | (0x00U << USART_CR3_CTSE_Pos))
#define USART_HardwareFlowControl_AutoFlowEn  	((0x01U << USART_CR3_RTSE_Pos) | (0x01U << USART_CR3_CTSE_Pos))

/** @defgroup USART_Interrupt_definition
* @{
*/
#define USART_IT_IDLEIEN           (0x01U << USART_CR1_IDLEIEN_Pos)        /*!< IDLE interrupt enable */
#define USART_IT_RXNEIEN           (0x01U << USART_CR1_RXNEIEN_Pos)        /*!< RXNE interrupt enable */
#define USART_IT_TCIEN             (0x01U << USART_CR1_TCIEN_Pos)          /*!< Transmission complete Interrupt enable */
#define USART_IT_TXEIEN            (0x01U << USART_CR1_TXEIEN_Pos)         /*!< TXE interrupt enable */
#define USART_IT_PEIEN        	   (0x01U << USART_CR1_PEIEN_Pos)          /*!< PE interrupt enable */
#define USART_IT_ERRIEN            (0x01U << USART_CR3_ERRIEN_Pos)         /*!< Error interrupt enable */
#define USART_IT_CTSIEN            (0x01U << USART_CR3_CTSE_Pos)           /*!< CTS enable */

/**
* @}
*/

/** @defgroup USART_DMA_Requests
* @{
*/
#define USART_DMAReq_EN           (0x01U << USART_CR3_DMAMODE_Pos)        /*!< DMA mode enable bit */

/**
* @}
*/

/** @defgroup USART_Flags
* @{
*/
#define USART_FLAG_PE      (0x01U << USART_SR_PE_Pos)    /*!< Parity error */
#define USART_FLAG_FE	   (0x01U << USART_SR_FE_Pos)    /*!< Framing error*/
#define USART_FLAG_NF	   (0x01U << USART_SR_NF_Pos)    /*!< Noise detected flag */
#define USART_FLAG_ORE	   (0x01U << USART_SR_ORE_Pos)   /*!< Overrun error */
#define USART_FLAG_IDLE	   (0x01U << USART_SR_IDLE_Pos)  /*!< IDLE frame detected */
#define USART_FLAG_RXNE    (0x01U << USART_SR_RXNE_Pos)  /*!< Read data register not empty */
#define USART_FLAG_TC      (0x01U << USART_SR_TC_Pos)    /*!< Transmission complete */
#define USART_FLAG_TXE     (0x01U << USART_SR_TXE_Pos)   /*!< Transmit data register Empty */
#define USART_FLAG_CTS	   (0x01U << USART_SR_CTS_Pos)   /*!< CTS mark*/

/** @defgroup USART_IT_Flags
* @{
*/
#define USART_IT_PE        (0x01U << USART_SR_PE_Pos)    /*!< Parity error */
#define USART_IT_FE		   (0x01U << USART_SR_FE_Pos)    /*!< Framing error*/
#define USART_IT_NF		   (0x01U << USART_SR_NF_Pos)    /*!< Noise detected flag */
#define USART_IT_ORE       (0x01U << USART_SR_ORE_Pos)   /*!< Overrun error */
#define USART_IT_IDLE	   (0x01U << USART_SR_IDLE_Pos)  /*!< IDLE frame detected */
#define USART_IT_RXNE      (0x01U << USART_SR_RXNE_Pos)  /*!< Read data register not empty */
#define USART_IT_TC        (0x01U << USART_SR_TC_Pos)    /*!< Transmission complete */
#define USART_IT_TXE       (0x01U << USART_SR_TXE_Pos)   /*!< Transmit data register Empty */
#define USART_IT_CTS	   (0x01U << USART_SR_CTS_Pos)   /*!< CTS mark*/

/* USART SYNC MODE */
#define USART_SYNC_MODE0	  ((0x00U << USART_CR2_CPHA_Pos) | (0x00U << USART_CR2_CPOL_Pos)) 
#define USART_SYNC_MODE1	  ((0x01U << USART_CR2_CPHA_Pos) | (0x00U << USART_CR2_CPOL_Pos)) 
#define USART_SYNC_MODE2	  ((0x00U << USART_CR2_CPHA_Pos) | (0x01U << USART_CR2_CPOL_Pos)) 
#define USART_SYNC_MODE3	  ((0x01U << USART_CR2_CPHA_Pos) | (0x01U << USART_CR2_CPOL_Pos)) 

/**
* @}
*/

void USART_DeInit(USART_TypeDef *usart);
void USART_Init(USART_TypeDef *usart, USART_InitTypeDef* init_struct);
void USART_StructInit(USART_InitTypeDef* init_struct);
void USART_Cmd(USART_TypeDef *usart, FunctionalState state);
void USART_ITConfig(USART_TypeDef *usart, uint16_t usart_it, FunctionalState state);
void USART_DMACmd(USART_TypeDef *usart, FunctionalState state);
void USART_SendData(USART_TypeDef *usart, uint16_t data);
uint16_t USART_ReceiveData(USART_TypeDef *usart);
FlagStatus USART_GetFlagStatus(USART_TypeDef *usart, uint16_t usart_flag);
void USART_ClearFlag(USART_TypeDef *usart, uint16_t usart_flag);
ITStatus USART_GetITStatus(USART_TypeDef *usart, uint16_t usart_it);
void USART_ClearITPendingBit(USART_TypeDef *usart, uint16_t usart_it);
void USART_SyncConfigure(USART_TypeDef *usart, uint8_t mode, uint32_t usart_bound);
void USART_SyncSlaveConfigure(USART_TypeDef *usart, uint8_t mode);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

/** --------------------------------------------------------------------------*/
#endif /* __HAL_UART_H -------------------------------------------------------*/
/** --------------------------------------------------------------------------*/
