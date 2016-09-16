/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "imu.h"
#include <stdlib.h>
#include <math.h>
#include "time.h"
/* Private define ------------------------------------------------------------*/
void write_buff( float *DataBuff ,float WriteData , int BuffLen  ); 
/* Private functions ---------------------------------------------------------*/

/*外部变量*/




void data_bubble(long double *DataBuff)
{
    long double Temp = 0 ; 
    for (int i = 0 ; i <= sizeof(DataBuff) + 1 ; i ++)
    {
        for ( int j = sizeof(DataBuff) +1  ; j > i ; j -- ) // 冒泡排序
        {
            if ( DataBuff[j] < DataBuff[j-1] ) 
            {
                Temp = DataBuff[j] ; 
                DataBuff[j] = DataBuff[j-1] ; 
                DataBuff[j-1] = Temp ; 
            }
        }   
    }
}

void write_buff( float *DataBuff ,float WriteData , int BuffLen  )  // 此处形参要传递指针  不能是数组
{
 
    
    BuffLen = 9 ; 
    for (int i=0 ; i <= BuffLen  ;i++) 
    {
        if ( ( i == BuffLen) && ( 0 != DataBuff[i] ) ) // 数据溢出，删除第一个数据
        {
            for ( int32_t j=1 ; j <= i ; j++ )
            {
                DataBuff[j-1] = DataBuff[j] ;     
            }
            DataBuff[i] = 0 ;
        }
        
        if ( 0 == DataBuff[i] )
        {
            DataBuff[i] = WriteData ; //　写入数据
        }
    }
    
}


void amplitude_smooth(long double DataBuff[]) // 幅值滤波
{
    uint8_t dataDisFlag = 1 ; 
    for (uint32_t i = 1 ; i <= sizeof(DataBuff) ; i ++)
    {
        if ( ( 50 < ( fabs(DataBuff[i]) - fabs(DataBuff[i+1])) ) && ( 50 < ( fabs(DataBuff[i]) - fabs(DataBuff[i-1])) ) ) // 发生干扰
        {
            DataBuff[i] = ( DataBuff[i+1] + DataBuff[i-1] ) / 2 ;   // 去除干扰数据 数据跟新为前一次和后一次的数据平均值
        }
    }  
}

long double data_mean( long double *DataBuff )
{
    uint32_t dataNum = 0 ; 
    long double data = 0 ; 
    long double dataTemp = 0 ; 
    for (uint32_t i = 1 ; i <= sizeof(DataBuff) + 1 ; i ++)
    {
        if ( 0 != DataBuff[i] )
        {
            dataTemp += DataBuff[i] ; 
            dataNum ++ ; 
        }
    }
    data = dataTemp / dataNum ; 
    
    return data ; 
}




void smooth_buff(long double *DataBuff , long double *data )
{
    long double SmootData = 0 ; 
       
    data_bubble(DataBuff) ; // 排序
    
    amplitude_smooth(DataBuff) ; // 幅值滤波
   
    SmootData = data_mean(DataBuff) ; // 平均值
    
    *data = SmootData ; 
    //return SmootData ; // 返回平均值

       //  MAGNStepAngle = MAGNSunAngle -MAGNLastAngle ; // 累计的角度-前一次角度=单步角度
         //   MAGNLastAngle = MAGNSunAngle ; // 这一次累计的角度等于前一次角度
         //   StepAngleProFlag = 1 ; //  
}