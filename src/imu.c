/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "imu.h"
#include <stdlib.h>
#include <math.h>
#include "time.h"
/* Private define ------------------------------------------------------------*/




/* Private functions ---------------------------------------------------------*/
/*标志位*/
/*数值*/
/*外部变量*/

extern int32_t IMUDataBuff[] ; 





void imu_data_pro(void) // imu数据处理 实时性 A
{
    uint16_t ImuRevData = 0 ; 
    uint16_t symbol = 0 ; 
    
    ImuRevData = SSP_SendByte(0x0400); //// 陀螺仪x轴命令                                  1
    ImuRevData &= ( ~( 1 << 15 ) ); // 压力低位数据数据
    ImuRevData &= ( ~( 1 << 14 ) );
    ImuRevData &= ( ~( 1 << 13 ) );
    ImuRevData &= ( ~( 1 << 12 ) );
    ImuRevData &= ( ~( 1 << 11 ) );
    ImuRevData &= ( ~( 1 << 10 ) );
    ImuRevData &= ( ~( 1 << 9 ) );
    ImuRevData &= ( ~( 1 << 8 ) );
    IMUDataBuff[PRESSURE_L] = ImuRevData ; 
        
    ImuRevData = SSP_SendByte(0x0600); //// 陀螺仪y轴命令                                  2
    ImuRevData &= ( ~( 1 << 14 ) ); // 陀螺仪x轴数据
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
    
    ImuRevData = SSP_SendByte(0x0800); //// 陀螺仪z轴命令                                  3
    ImuRevData &= ( ~( 1 << 14 ) ); // 陀螺仪y轴数据
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
        
    ImuRevData = SSP_SendByte(0x0A00); //// 加速度x轴命令                                  4
    ImuRevData &= ( ~( 1 << 14 ) ); // 陀螺仪z轴数据
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
    
    ImuRevData = SSP_SendByte(0x0C00); //// 加速度y轴命令                                  5 
    ImuRevData &= ( ~( 1 << 14 ) ); // 加速度x轴数据
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
    
    ImuRevData = SSP_SendByte(0x0E00); //// 加速度z轴命令                                  6   
    ImuRevData &= ( ~( 1 << 14 ) ); // 加速度y轴数据
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

    ImuRevData = SSP_SendByte(0x1000); //// 地磁x轴命令                                    7 
    ImuRevData &= ( ~( 1 << 14 ) ); // 加速度z轴数据
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
    
    ImuRevData = SSP_SendByte(0x1200); //// 地磁y轴命令                                    8  
    ImuRevData &= ( ~( 1 << 14 ) ); // 地磁x轴数据
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
        
    ImuRevData = SSP_SendByte(0x1400); //// 地磁z轴命令                                    9    
    ImuRevData &= ( ~( 1 << 14 ) ); // 地磁y轴数据
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
        
    ImuRevData = SSP_SendByte(0x1600); // 压力高位命令                                    10
    ImuRevData &= ( ~( 1 << 14 ) ); // 地磁z轴数据
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

    ImuRevData = SSP_SendByte(0x1800); // 压力低位命令                                    11
    ImuRevData &= ( ~( 1 << 14 ) ); // 压力高位数据数据
    ImuRevData &= ( ~( 1 << 15 ) );
    IMUDataBuff[PRESSURE] = ImuRevData ;        
}



