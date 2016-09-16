/************************************************************************************
* File Name          : init.c
* Date First Issued  : 8/11/2012
* Description        : RHJ450-C/A ����Ա��������ʼ������.
***************************************************************************************/

/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "head.h"
#include "time.h"

extern uint8_t ImuDataProLabel ; // imu���ݴ����־λ
extern uint8_t AccelDataProLabel ; // ���ٶ����ݴ���
extern uint8_t MAGNDataProLabel ; // �ش����ݴ���
extern uint8_t GyroDataProLabel ; // ���������ݴ���
extern uint8_t MAGNAngleProLabel ; // �����Ƕȴ����־λ

uint8_t UartSendTimerLabel = 0 ; 
uint32_t UartSendTimerValue = 0 ; 
uint16_t SetpCycleValue = 0 ; 

int MAGNAngleProTimeVal = 0 ; 

extern uint8_t StepDistinDelaySw ;
extern uint32_t StepDistinDelayTimer ;


void TIMER16_0_IRQHandler(void) // 16λ��ʱ��0�жϷ�����
{
    uint32_t  int_status_reg;

    int_status_reg = TIM16_GetIntStatusReg (LPC_CT16B0);

    /*clear the interrupt flag of match channel 0 */
    if (int_status_reg & TIM16_INT_MR0)	
    {
    	TIM16_ClearIntPendingBit(LPC_CT16B0, TIM16_INT_MR0);
        
        /*�жϷ�����*/
        
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




void TIMER16_1_IRQHandler(void) // 16λ��ʱ��1�жϷ�����
{
    uint32_t  int_status_reg;

    int_status_reg = TIM16_GetIntStatusReg (LPC_CT16B1);

    /*clear the interrupt flag of match channel 0 */
    if (int_status_reg & TIM16_INT_MR0)	
    {
    	TIM16_ClearIntPendingBit(LPC_CT16B1, TIM16_INT_MR0);
        /*�жϷ�����*/
       
        // GPIO_SetLowLevel(LPC_GPIO0, 10, 1);  // ����Ƶ��
        
        ImuDataProLabel = 1 ; 
        
        AccelDataProLabel = 1 ; 
        
        MAGNDataProLabel = 1 ; 
        
        GyroDataProLabel = 1 ;   
        
        imu_data_pro() ; 
        accel_data_pro() ; 
        gyro_data_pro() ;   
     
        MAGN_data_pro() ;    
        
        //GPIO_SetHighLevel(LPC_GPIO0, 10, 1);  // ����Ƶ��
        
        /*
        if ( SETP_CYCLE_MAX > SetpCycleValue )
        {
            SetpCycleValue ++ ; 
        }
        
        */
 
        
        if ( StepDistinDelaySw ) // ����ʶ����ʱ
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