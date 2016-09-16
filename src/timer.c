/************************************************************************************
* File Name          : init.c
* Date First Issued  : 8/11/2012
* Description        : RHJ450-C/A 消防员呼救器初始化程序.
***************************************************************************************/

/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "head.h"
#include "time.h"

extern uint8_t ImuDataProLabel ; // imu数据处理标志位
extern uint8_t AccelDataProLabel ; // 加速度数据处理
extern uint8_t MAGNDataProLabel ; // 地磁数据处理
extern uint8_t GyroDataProLabel ; // 角速率数据处理
extern uint8_t MAGNAngleProLabel ; // 磁力角度处理标志位

uint8_t UartSendTimerLabel = 0 ; 
uint32_t UartSendTimerValue = 0 ; 
uint16_t SetpCycleValue = 0 ; 

int MAGNAngleProTimeVal = 0 ; 

extern uint8_t StepDistinDelaySw ;
extern uint32_t StepDistinDelayTimer ;


void TIMER16_0_IRQHandler(void) // 16位定时器0中断服务函数
{
    uint32_t  int_status_reg;

    int_status_reg = TIM16_GetIntStatusReg (LPC_CT16B0);

    /*clear the interrupt flag of match channel 0 */
    if (int_status_reg & TIM16_INT_MR0)	
    {
    	TIM16_ClearIntPendingBit(LPC_CT16B0, TIM16_INT_MR0);
        
        /*中断服务函数*/
        
        if ( 20 > UartSendTimerValue )
        {
            UartSendTimerValue ++ ;     
        }
        else
        {
            UartSendTimerLabel = 1 ; 
        }
        
        
        
    }
}




void TIMER16_1_IRQHandler(void) // 16位定时器1中断服务函数
{
    uint32_t  int_status_reg;

    int_status_reg = TIM16_GetIntStatusReg (LPC_CT16B1);

    /*clear the interrupt flag of match channel 0 */
    if (int_status_reg & TIM16_INT_MR0)	
    {
    	TIM16_ClearIntPendingBit(LPC_CT16B1, TIM16_INT_MR0);
        /*中断服务函数*/
       
        // GPIO_SetLowLevel(LPC_GPIO0, 10, 1);  // 测试频率
        
        ImuDataProLabel = 1 ; 
        
        AccelDataProLabel = 1 ; 
        
        MAGNDataProLabel = 1 ; 
        
        GyroDataProLabel = 1 ;   
        
        imu_data_pro() ; 
        accel_data_pro() ; 
        gyro_data_pro() ;   
     
        MAGN_data_pro() ;    
        
        //GPIO_SetHighLevel(LPC_GPIO0, 10, 1);  // 测试频率
        
        /*
        if ( SETP_CYCLE_MAX > SetpCycleValue )
        {
            SetpCycleValue ++ ; 
        }
        
        */
 
        
        if ( StepDistinDelaySw ) // 单步识别延时
	{
	    if ( 70000 > StepDistinDelayTimer )
            {
                StepDistinDelayTimer ++ ; // 
            }
	}
	else
	{
	    StepDistinDelayTimer = 0 ; //  
	}
        
    }
}