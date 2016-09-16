/************************************************************************************
* File Name          : init.c
* Date First Issued  : 
* Description        :
***************************************************************************************/

/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "head.h"


#define UART_PORT 1

#if (UART_PORT == 0)
#define TEST_UART LPC_UART0
#define TEST_UART_RXD  IOCON_UART_RXD0_LOC0
#define TEST_UART_TXD  IOCON_UART_TXD0_LOC0

#elif (UART_PORT == 1)
#define TEST_UART (LPC_UART_TypeDef *)LPC_UART1
#define TEST_UART_RXD  IOCON_UART_RXD1_LOC0
#define TEST_UART_TXD  IOCON_UART_TXD1_LOC0
#endif

TIM16_InitTypeDef TIM16_InitStruct;
TIM16_MATCHTypeDef TIM16_MatchConfigStruct ;
IOCON_PIO_CFG_Type PIO_ConfigStructure;

static SSP_InitTypeDef SSP_InitStructure;
static IOCON_PIO_CFG_Type PIO_ConfigStructure;


void timer16_0_init(void) ; 
void timer16_1_init(void) ; 
void SPI_init(void) ;
void USART_init(void) ; 
void GPIO_init(void) ; 
void GPIO_power_init(void) ; 
void GPIO_led_init(void) ; 
void GPIO_key_init(void) ; 


/* Private variables ---------------------------------------------------------*/




void init_devices() //¡¡³õÊ¼»¯º¯Êý
{
    SystemInit() ;
    timer16_0_init() ;
    timer16_1_init() ;
    SPI_init() ; 
    //SYS_ConfigAHBCLK (SYS_AHBCLKCTRL_IOCON, ENABLE);    /* Enable clock to IOCON*/
    USART_init() ; 
    GPIO_init() ; 

    
}


void GPIO_init(void)
{
    IOCON_PIO_CFG_Type PIO_mode;

    SYS_ConfigAHBCLK(SYS_AHBCLKCTRL_GPIO2, ENABLE);		
    
    IOCON_StructInit(&PIO_mode);
    
    PIO_mode.type = IOCON_PIO_1_0;									 //PIO_mode.type = IOCON_PIO_0_22-->PIO_mode.type = IOCON_PIO_2_0;
    IOCON_SetFunc(&PIO_mode); 
    PIO_mode.type = IOCON_PIO_1_1;									 //PIO_mode.type = IOCON_PIO_0_22-->PIO_mode.type = IOCON_PIO_2_0;
    IOCON_SetFunc(&PIO_mode); 
    PIO_mode.type = IOCON_PIO_1_2;									 //PIO_mode.type = IOCON_PIO_0_22-->PIO_mode.type = IOCON_PIO_2_0;
    IOCON_SetFunc(&PIO_mode); 
    
    PIO_mode.type = IOCON_PIO_0_10; //test									 //PIO_mode.type = IOCON_PIO_0_22-->PIO_mode.type = IOCON_PIO_2_0;
    IOCON_SetFunc(&PIO_mode); 
    GPIO_SetDir(LPC_GPIO0, 10, OUTPUT);
    GPIO_SetHighLevel(LPC_GPIO0, 10, 1);  
    GPIO_SetLowLevel(LPC_GPIO0, 10, 1);   
}



/**
  * @brief  timer16_0 init
  *
  * @param  10hz-0.1s timer
  * @retval None
  */
void timer16_0_init(void)
{
    /* Timer16B0 I/O config: match channel 0 on location0 (PIO0_11) */
    //IOCON_StructInit (&PIO_ConfigStructure);
    PIO_ConfigStructure.type = IOCON_CT16_B0_MAT0_LOC0;
    IOCON_SetFunc (&PIO_ConfigStructure);

    
    /* Initialize timer16B0, prescale count time of 10uS */
    TIM16_InitStruct.PrescaleOption = TIM16_PRESCALE_USVAL;
    TIM16_InitStruct.PrescaleValue = 100;
    
    /* Match channel 0, MR0 */
    TIM16_MatchConfigStruct.MatchChannel = TIM16_MATCH_CHANNEL0;
    /* Enable interrupt when MR0 matches the value in TC register */
    TIM16_MatchConfigStruct.IntOnMatch   = TRUE;
    /* Enable reset on MR0: TIMER will reset if MR0 matches it */
    TIM16_MatchConfigStruct.ResetOnMatch = TRUE;
    /* No Stop on MR0 if MR0 matches it */
    TIM16_MatchConfigStruct.StopOnMatch  = FALSE;
    /* Toggle MR0 pin if MR0 matches it */
    TIM16_MatchConfigStruct.ExtMatchOutputType =TIM16_EXTMATCH_TOGGLE;
    /* Set Match value, count value of 100 (1000 * 100uS = 100mS --> 10Hz) */
    TIM16_MatchConfigStruct.MatchValue   = 1000;
    
    /* Set configuration for Tim16_config and Tim16_MatchConfig */
    TIM16_Init(LPC_CT16B0, TIM16_TIMER_MODE,&TIM16_InitStruct);
    TIM16_ConfigMatch(LPC_CT16B0,&TIM16_MatchConfigStruct);

    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(TIMER_16_0_IRQn, ((0x01<<3)|0x01)); 
    /* Enable interrupt for timer */
    NVIC_EnableIRQ(TIMER_16_0_IRQn);
    /* To start timer16B0 */
    TIM16_Cmd(LPC_CT16B0, ENABLE);
}


/**
  * @brief  timer16_1 init
  *
  * @param  1000hz-0.001ms timer
  * @retval None
  */
void timer16_1_init(void)
{
    /* Timer16B0 I/O config: match channel 0 on location0 (PIO0_11) */
    //IOCON_StructInit (&PIO_ConfigStructure);
    PIO_ConfigStructure.type = IOCON_CT16_B1_MAT0_LOC0;
    IOCON_SetFunc (&PIO_ConfigStructure);

    
    /* Initialize timer16B0, prescale count time of 10uS */
    TIM16_InitStruct.PrescaleOption = TIM16_PRESCALE_USVAL;
    TIM16_InitStruct.PrescaleValue = 50;
    
    /* Match channel 0, MR0 */
    TIM16_MatchConfigStruct.MatchChannel = TIM16_MATCH_CHANNEL0;
    /* Enable interrupt when MR0 matches the value in TC register */
    TIM16_MatchConfigStruct.IntOnMatch   = TRUE;
    /* Enable reset on MR0: TIMER will reset if MR0 matches it */
    TIM16_MatchConfigStruct.ResetOnMatch = TRUE;
    /* No Stop on MR0 if MR0 matches it */
    TIM16_MatchConfigStruct.StopOnMatch  = FALSE;
    /* Toggle MR0 pin if MR0 matches it */
    TIM16_MatchConfigStruct.ExtMatchOutputType =TIM16_EXTMATCH_TOGGLE;
    /* Set Match value, count value of 100 (100 * 50uS = 5mS --> 200Hz) */
    TIM16_MatchConfigStruct.MatchValue   = 100;
    
    /* Set configuration for Tim16_config and Tim16_MatchConfig */
    TIM16_Init(LPC_CT16B1, TIM16_TIMER_MODE,&TIM16_InitStruct);
    TIM16_ConfigMatch(LPC_CT16B1,&TIM16_MatchConfigStruct);

    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(TIMER_16_1_IRQn, ((0x01<<3)|0x01)); 
    /* Enable interrupt for timer */
    NVIC_EnableIRQ(TIMER_16_1_IRQn);
    /* To start timer16B0 */
    TIM16_Cmd(LPC_CT16B1, ENABLE);
}


void SPI_init(void)
{
    /*  SSP I/O config: CLK, SSEL, MISO, MOSI */
    IOCON_StructInit (&PIO_ConfigStructure);
    PIO_ConfigStructure.type = IOCON_SSP_SCK;
    IOCON_SetFunc (&PIO_ConfigStructure);
    PIO_ConfigStructure.type = IOCON_SSP_SSEL;
    IOCON_SetFunc (&PIO_ConfigStructure);
    PIO_ConfigStructure.type = IOCON_SSP_MISO;
    IOCON_SetFunc (&PIO_ConfigStructure);
    PIO_ConfigStructure.type = IOCON_SSP_MOSI;
    IOCON_SetFunc (&PIO_ConfigStructure);
 
    /* Initialize SSP configuration structure with user specified parameter */
    SSP_InitStructure.DataSize = SSP_DATASIZE_16;
    SSP_InitStructure.FrameFormat = SSP_FRAMEFORMAT_SPI;
    SSP_InitStructure.CPOL = SSP_CPOL_HIGH;
    SSP_InitStructure.CPHA = SSP_CPHA_SECOND;//SSP_CPHA_FIRST;
    SSP_InitStructure.Mode = SSP_MODE_MASTER;
    SSP_InitStructure.ClockRate = 1000000;
    SSP_Init(&SSP_InitStructure);

    /* Enable SSP peripheral */
    SSP_Cmd (ENABLE);
    
}

void USART_init()
{
    /* Initialize debug via UART1, 9600-8-N-1-N */
    DB_Init ();
}