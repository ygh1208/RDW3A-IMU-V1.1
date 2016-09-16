/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "imu.h"
#include <stdlib.h>
#include <math.h>
#include "time.h"
/* Private define ------------------------------------------------------------*/
void write_buff( float *DataBuff ,float WriteData , int BuffLen  ); 
/* Private functions ---------------------------------------------------------*/

/*�ⲿ����*/




void data_bubble(long double *DataBuff)
{
    long double Temp = 0 ; 
    for (int i = 0 ; i <= sizeof(DataBuff) + 1 ; i ++)
    {
        for ( int j = sizeof(DataBuff) +1  ; j > i ; j -- ) // ð������
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

void write_buff( float *DataBuff ,float WriteData , int BuffLen  )  // �˴��β�Ҫ����ָ��  ����������
{
 
    
    BuffLen = 9 ; 
    for (int i=0 ; i <= BuffLen  ;i++) 
    {
        if ( ( i == BuffLen) && ( 0 != DataBuff[i] ) ) // ���������ɾ����һ������
        {
            for ( int32_t j=1 ; j <= i ; j++ )
            {
                DataBuff[j-1] = DataBuff[j] ;     
            }
            DataBuff[i] = 0 ;
        }
        
        if ( 0 == DataBuff[i] )
        {
            DataBuff[i] = WriteData ; //��д������
        }
    }
    
}


void amplitude_smooth(long double DataBuff[]) // ��ֵ�˲�
{
    uint8_t dataDisFlag = 1 ; 
    for (uint32_t i = 1 ; i <= sizeof(DataBuff) ; i ++)
    {
        if ( ( 50 < ( fabs(DataBuff[i]) - fabs(DataBuff[i+1])) ) && ( 50 < ( fabs(DataBuff[i]) - fabs(DataBuff[i-1])) ) ) // ��������
        {
            DataBuff[i] = ( DataBuff[i+1] + DataBuff[i-1] ) / 2 ;   // ȥ���������� ���ݸ���Ϊǰһ�κͺ�һ�ε�����ƽ��ֵ
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
       
    data_bubble(DataBuff) ; // ����
    
    amplitude_smooth(DataBuff) ; // ��ֵ�˲�
   
    SmootData = data_mean(DataBuff) ; // ƽ��ֵ
    
    *data = SmootData ; 
    //return SmootData ; // ����ƽ��ֵ

       //  MAGNStepAngle = MAGNSunAngle -MAGNLastAngle ; // �ۼƵĽǶ�-ǰһ�νǶ�=�����Ƕ�
         //   MAGNLastAngle = MAGNSunAngle ; // ��һ���ۼƵĽǶȵ���ǰһ�νǶ�
         //   StepAngleProFlag = 1 ; //  
}