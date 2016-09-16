/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "imu.h"
#include <stdlib.h>
#include <math.h>
#include "time.h"
/* Private define ------------------------------------------------------------*/




/* Private functions ---------------------------------------------------------*/
/*��־λ*/
/*��ֵ*/
/*�ⲿ����*/

extern int32_t IMUDataBuff[] ; 





void imu_data_pro(void) // imu���ݴ��� ʵʱ�� A
{
    uint16_t ImuRevData = 0 ; 
    uint16_t symbol = 0 ; 
    
    ImuRevData = SSP_SendByte(0x0400); //// ������x������                                  1
    ImuRevData &= ( ~( 1 << 15 ) ); // ѹ����λ��������
    ImuRevData &= ( ~( 1 << 14 ) );
    ImuRevData &= ( ~( 1 << 13 ) );
    ImuRevData &= ( ~( 1 << 12 ) );
    ImuRevData &= ( ~( 1 << 11 ) );
    ImuRevData &= ( ~( 1 << 10 ) );
    ImuRevData &= ( ~( 1 << 9 ) );
    ImuRevData &= ( ~( 1 << 8 ) );
    IMUDataBuff[PRESSURE_L] = ImuRevData ; 
        
    ImuRevData = SSP_SendByte(0x0600); //// ������y������                                  2
    ImuRevData &= ( ~( 1 << 14 ) ); // ������x������
    ImuRevData &= ( ~( 1 << 15 ) );
    symbol =  ( ImuRevData & ( 1 << 13 ) );
    if( 0 == symbol )
    {   
        IMUDataBuff[GYRO_X] = ImuRevData ;             
    }
    else 
    {
        ImuRevData = ~ImuRevData; 
        ImuRevData &= ( ~( 1 << 14 ) );
        ImuRevData &= ( ~( 1 << 15 ) );
        IMUDataBuff[GYRO_X] = ImuRevData * (-1) ;       
    }
    
    ImuRevData = SSP_SendByte(0x0800); //// ������z������                                  3
    ImuRevData &= ( ~( 1 << 14 ) ); // ������y������
    ImuRevData &= ( ~( 1 << 15 ) );
    symbol =  ( ImuRevData & ( 1 << 13 ) );
    if( 0 == symbol )
    {
        IMUDataBuff[GYRO_Y] = ImuRevData ;   
    }
    else 
    {
        ImuRevData = ~ImuRevData; 
        ImuRevData &= ( ~( 1 << 14 ) );
        ImuRevData &= ( ~( 1 << 15 ) );
        IMUDataBuff[GYRO_Y] = ImuRevData * (-1) ;  
    }
       //DB_PutStr(" ");
        
    ImuRevData = SSP_SendByte(0x0A00); //// ���ٶ�x������                                  4
    ImuRevData &= ( ~( 1 << 14 ) ); // ������z������
    ImuRevData &= ( ~( 1 << 15 ) );
    symbol =  ( ImuRevData & ( 1 << 13 ) );
    if( 0 == symbol )
    {
        IMUDataBuff[GYRO_Z] = ImuRevData ;   
    }
    else 
    {
        ImuRevData = ~ImuRevData; 
        ImuRevData &= ( ~( 1 << 14 ) );
        ImuRevData &= ( ~( 1 << 15 ) );
        IMUDataBuff[GYRO_Z] = ImuRevData * (-1) ;   
    }
    
    ImuRevData = SSP_SendByte(0x0C00); //// ���ٶ�y������                                  5 
    ImuRevData &= ( ~( 1 << 14 ) ); // ���ٶ�x������
    ImuRevData &= ( ~( 1 << 15 ) );
    symbol =  ( ImuRevData & ( 1 << 13 ) );
    if( 0 == symbol )
    {
        IMUDataBuff[ACCEL_X] = ImuRevData ;   
    }
    else 
    {
        ImuRevData = ~ImuRevData; 
        ImuRevData &= ( ~( 1 << 14 ) );
        ImuRevData &= ( ~( 1 << 15 ) );
        IMUDataBuff[ACCEL_X] = ImuRevData * (-1);   
    }
    
    ImuRevData = SSP_SendByte(0x0E00); //// ���ٶ�z������                                  6   
    ImuRevData &= ( ~( 1 << 14 ) ); // ���ٶ�y������
    ImuRevData &= ( ~( 1 << 15 ) );
    symbol =  ( ImuRevData & ( 1 << 13 ) );
    if( 0 == symbol )
    {   
        IMUDataBuff[ACCEL_Y] = ImuRevData ;   
    }
    else 
    {
        ImuRevData = ~ImuRevData; 
        ImuRevData &= ( ~( 1 << 14 ) );
        ImuRevData &= ( ~( 1 << 15 ) );
        IMUDataBuff[ACCEL_Y] = ImuRevData * (-1);      
    }

    ImuRevData = SSP_SendByte(0x1000); //// �ش�x������                                    7 
    ImuRevData &= ( ~( 1 << 14 ) ); // ���ٶ�z������
    ImuRevData &= ( ~( 1 << 15 ) );
    symbol =  ( ImuRevData & ( 1 << 13 ) );
    if( 0 == symbol )
    {   
        IMUDataBuff[ACCEL_Z] = ImuRevData ;   
    }
    else 
    {
        ImuRevData = ~ImuRevData; 
        ImuRevData &= ( ~( 1 << 14 ) );
        ImuRevData &= ( ~( 1 << 15 ) );
        IMUDataBuff[ACCEL_Z] = ImuRevData * (-1) ;   
    }
    
    ImuRevData = SSP_SendByte(0x1200); //// �ش�y������                                    8  
    ImuRevData &= ( ~( 1 << 14 ) ); // �ش�x������
    ImuRevData &= ( ~( 1 << 15 ) );
    symbol =  ( ImuRevData & ( 1 << 13 ) );
    if( 0 == symbol )
    {   
        IMUDataBuff[MAGN_X] = ImuRevData ;              
    }
    else 
    {
        ImuRevData = ~ImuRevData; 
        ImuRevData &= ( ~( 1 << 14 ) );
        ImuRevData &= ( ~( 1 << 15 ) );
        IMUDataBuff[MAGN_X] = ImuRevData * (-1);      
    } 
        
    ImuRevData = SSP_SendByte(0x1400); //// �ش�z������                                    9    
    ImuRevData &= ( ~( 1 << 14 ) ); // �ش�y������
    ImuRevData &= ( ~( 1 << 15 ) );
    symbol =  ( ImuRevData & ( 1 << 13 ) );
    if( 0 == symbol )
    {   
        IMUDataBuff[MAGN_Y] = ImuRevData ;        
    }
    else 
    {
        ImuRevData = ~ImuRevData; 
        ImuRevData &= ( ~( 1 << 14 ) );
        ImuRevData &= ( ~( 1 << 15 ) );
        IMUDataBuff[MAGN_Y] = ImuRevData * (-1);    
    }
        
    ImuRevData = SSP_SendByte(0x1600); // ѹ����λ����                                    10
    ImuRevData &= ( ~( 1 << 14 ) ); // �ش�z������
    ImuRevData &= ( ~( 1 << 15 ) );
    symbol =  ( ImuRevData & ( 1 << 13 ) );
    if( 0 == symbol )
    {   
        IMUDataBuff[MAGN_Z] = ImuRevData ;        
    }
    else 
    {
        ImuRevData = ~ImuRevData; 
        ImuRevData &= ( ~( 1 << 14 ) );
        ImuRevData &= ( ~( 1 << 15 ) );
        IMUDataBuff[MAGN_Z] = ImuRevData * (-1);        
    }

    ImuRevData = SSP_SendByte(0x1800); // ѹ����λ����                                    11
    ImuRevData &= ( ~( 1 << 14 ) ); // ѹ����λ��������
    ImuRevData &= ( ~( 1 << 15 ) );
    IMUDataBuff[PRESSURE] = ImuRevData ;        
}



