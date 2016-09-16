/************************************************************************************
* File Name          : init.c
* Date First Issued  : 8/11/2012
* Description        : RHJ450-C/A 消防员呼救器初始化程序.
***************************************************************************************/

/* Includes ------------------------------------------------------------------*/  

#include "lpc12xx_libcfg.h"
#include "head.h"

#define DATA_BLOCK_SIZE     128

static uint16_t buff_tx[DATA_BLOCK_SIZE], buff_rx[DATA_BLOCK_SIZE];

static uint32_t test_ret_code = 0;


uint16_t SSP_SendByte(uint16_t byte);
uint16_t SSP_RecvByte(void);

/**
  * @brief  Sends a byte through the SSP interface and return the byte received
  *         from the SSP bus.  
  *
  * @param  byte: byte to send.
  * @retval The value of the received byte.
  */
uint16_t SSP_SendByte(uint16_t byte)
{
    uint16_t byte_r;

    /* Clear Rx FIFO */
    while (SSP_GetStatus(SSP_STATUS_RNE) == SET)
        byte_r =  SSP_ReceiveData();

    /* Wait if Tx FIFO is not empty */ 
    while (SSP_GetStatus(SSP_STATUS_TFE) == RESET);    

    /* Send byte through the SSP peripheral */
    SSP_SendData(byte);

    /* Wait for transfer to finish */
    while (SSP_GetStatus(SSP_STATUS_BSY) == SET);

    /* Wait to receive a byte */
    while (SSP_GetStatus(SSP_STATUS_RNE) == RESET); 

    /* Return the byte read from the SSP bus */
    byte_r = (uint16_t)SSP_ReceiveData(); 

    return byte_r;
}

/**
  * @brief  Master reads a byte from the SSP bus.
  *
  * @param  None
  * @retval Byte Read from the SSP bus.
  */
uint16_t SSP_RecvByte(void)
{
    return (SSP_SendByte(0x1000));
}


