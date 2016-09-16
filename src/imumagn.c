/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "imu.h"
#include <stdlib.h>
#include <math.h>
#include "time.h"
/* Private define ------------------------------------------------------------*/
#define DATA_LEN     9
/* Private functions ---------------------------------------------------------*/

/*�ⲿ����*/
extern int32_t IMUDataBuff[] ; 
extern int32_t StepETmp ; 
extern uint8_t MAGNAngleSmoothGat ;


//extern long double MAGNTmp0 ; 

extern uint8_t StepAngleProFlag ; 


// �����Ʊ�������

long double MAGNAngleErr = 0 ;

long double Acceltmp = 0 ; 
long double abc = -4 ; 

long double AngleSmoothBuffTmp = 0 ; 
long double AngleSmoothBuffOld = 0 ; 



//////////////////

//float MAGNHi = 0 ; 
//float MAGNHnor = 0 ; 
float HiDataBuff[DATA_LEN+1] = {0} ; 
float HiSmoothData = 0 ; // 
float HnorDataBuff[DATA_LEN+1] = {0} ; 
static int32_t DataLen = 0 ; 
float HnroSmoothData = 0 ; 
float MAGNAngle = 0 ;
float MAGNAngleBuff[DATA_LEN+1] = 0 ; 
float MAGNAngleTemp = 0 ; // δ�˲��شŽǶ�ֵ
float MAGNSunAngleTmp = 0 ; // �ش��ۼƽǶ�ֵ

uint8_t HiSmoothFlag = 0 ; 
uint8_t HnorSmoothFlag = 0 ; 
uint8_t MAGNAngleSmoothFlag = 0 ; 


//////////
uint8_t MAGNAngleSmoothGat = 0 ; 
uint8_t MAGNAngleSmoothAdd = 0 ; 
uint8_t MAGNAngleSmoothAddTmp = 0 ; 


float InclineO = 0 ; 
float InclineR = 0 ; 


void MAGN_clear_buff( float *DataBuff , int BuffLen )
{
    for (int i=0 ; i <= BuffLen  ;i++) 
    {
        DataBuff[i] = 0 ; 
    }
}


void MAGN_write_buff( float *DataBuff ,float WriteData , int BuffLen  )  // �˴��β�Ҫ����ָ��  ����������
{
   
    for (int i=0 ; i <= BuffLen  ;i++) 
    {
      
       /*
        if ( ( i == BuffLen) && ( 0 != DataBuff[i] ) ) // ���������ɾ����һ������
        {
            for ( int32_t j=1 ; j <= i ; j++ )
            {
                DataBuff[j-1] = DataBuff[j] ;     
            }
          
            
            for ( int32_t k=0 ; k <= i ; k++ ) // ���������ɾ����������
            {
                DataBuff[k] = 0 ;
                i = 0 ;
            }
            
          // ���ݱ��� ��������
        }
        */ 
  
        if ( 0 == DataBuff[i] )
        {
            DataBuff[i] = WriteData ; //��д������
            break ; 
        }
    }
    
}



///////////////////////////////////////////////


int incline_scan(int32_t x, int32_t y)
{
    if ( ( ( x >= InclineO ) && ( y <= InclineO ) ) && ( ( x >= InclineR ) && ( y <= InclineR ) ) ) // ȡ�Ƕ�ֵʱ������ȡ���С��
    {
        return 1; 
    }  
    return 0 ; 
}


///////////////////////////////



void incline_pro() //���
{   
    float InclineRTmp = 0 ; 
    float InclineOTmp = 0 ; 
    float dataTempX = 0 ; 
    float dataTempY = 0 ;
    float dataTempZ = 0 ; 
    uint8_t dataErr1 = 0 ; 
    uint8_t dataErr2 = 0 ; 
  
    dataErr1 = 1 ; 
    dataErr2 = -1 ; 
    dataTempX = IMUDataBuff[ACCEL_X] ;
    InclineOTmp = dataTempX / 300 ;  // 
    
    if ( 1 < InclineOTmp ) // ��ֹ�������
    {
        InclineOTmp = dataErr1 ; 
    }
    if ( -1 > InclineOTmp )
    {
        InclineOTmp = dataErr2 ;
    }
    InclineO = asinl(InclineOTmp) * 180 / 3.14159 ;  
    
    dataTempY = IMUDataBuff[ACCEL_Y] ; 
    dataTempZ = IMUDataBuff[ACCEL_Z] ;   
    InclineRTmp = dataTempZ / dataTempY ; 
  
    InclineR = atanl(InclineRTmp) * 180 / 3.14159 ; 
}





float MAGN_Hi_pro()
{
    float MAGNHiTmp = 0 ;  
   
    MAGNHiTmp =  ( IMUDataBuff[MAGN_X] * IMUDataBuff[MAGN_X] )  
              + ( IMUDataBuff[MAGN_Y] * IMUDataBuff[MAGN_Y] ) 
              + ( IMUDataBuff[MAGN_Z] * IMUDataBuff[MAGN_Z] ) ;
    return sqrt(MAGNHiTmp) ; 
}

float MAGN_Hnor_pro()
{
    float MAGNHnorTmp = 0 ;
    
    MAGNHnorTmp =  ( IMUDataBuff[MAGN_X] * IMUDataBuff[MAGN_X] )  
              + ( IMUDataBuff[MAGN_Y] * IMUDataBuff[MAGN_Y] ) 
              + ( IMUDataBuff[MAGN_Z] * IMUDataBuff[MAGN_Z] ) ;
    return sqrt(MAGNHnorTmp) ; 
}

void MAGN_data_ari() 
{
    float MAGNTmp0 = 0 ; 
    float MAGNTmp1 = 0 ; 
    float MAGNTmp2 = 0 ; 
    float MAGNTmp3 = 0 ; 
    float MAGNTmp4 = 0 ; 
    float MAGNTmp5 = 0 ; 

    MAGNTmp0 = sin( InclineR * 3.14159/180 ) * ( IMUDataBuff[MAGN_Y] * 0.0005 ) ;
    MAGNTmp1 = cos( InclineR * 3.14159/180 ) * ( IMUDataBuff[MAGN_Z] * 0.0005 ) ;
    MAGNTmp2 = cos( InclineO * 3.14159/180 ) * ( IMUDataBuff[MAGN_X] * 0.0005 ) ;
    MAGNTmp3 = sin( InclineR * 3.14159/180 ) * sin( InclineO * 3.14159/180 ) * ( IMUDataBuff[MAGN_Z] * 0.0005 ) ; 
    MAGNTmp4 = sin( InclineO * 3.14159/180 ) * cos( InclineR * 3.14159/180 ) * ( IMUDataBuff[MAGN_Y] * 0.0005 ); 
    MAGNTmp5 = ( MAGNTmp0 - MAGNTmp1 ) / ( MAGNTmp2 + MAGNTmp3 + MAGNTmp4 ) ; 
    MAGNAngle = atan( MAGNTmp5 ) * 180/3.14159 ; 
}
   
    
void MAGN_data_pro() // �شŽǶȴ��� ʵʱ�� A
{ 
    float MAGNHi = 0 ; 
    float MAGNHnor = 0 ; 
    
    incline_pro() ; // ��Ǵ���
    MAGN_data_ari() ; // �㷨
    
    if(1) //�˶�״̬  
    { 
        MAGNHi = MAGN_Hi_pro() ; // 
        MAGN_write_buff( HiDataBuff , MAGNHi  ,DATA_LEN) ; // дhi  
    }
    
     if (1) //��ֹ״̬
    {
        MAGNHnor = MAGN_Hnor_pro() ;//
        MAGN_write_buff(HnorDataBuff , MAGNHnor , DATA_LEN ) ; // д hnor   
    } 
}




//////////////////////////////////////////////////////////////////////////////////////////////////////


int MAGN_data_saturation(float *DataBuff , int BuffLen)
{
    for ( int i=0 ; i <= BuffLen ; i++ )
    {
        if ( 0 == DataBuff[BuffLen] )
        {
            return 0 ; 
        }
    }
    return 1 ; 
}





void MAGN_data_bubble(float *DataBuff , int DataLen)
{
    float Temp = 0 ; 
    for (int i = 0 ; i <= DataLen ; i ++)
    {
        for ( int j = DataLen  ; j > i ; j -- ) // ð������
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


void MAGN_amplitude_smooth(float *DataBuff , int DataLen) // ��ֵ�˲�
{
    uint8_t dataDisFlag = 1 ; 
    //DataBuff[5] = 0 ; //test
    for (uint32_t i = 1 ; i <= DataLen ; i ++)
    {
        if ( ( 50 <  fabs( DataBuff[i] - DataBuff[i+1] ) ) && ( 50 < fabs( DataBuff[i] - DataBuff[i-1] ) ) ) // ��������
        {
            DataBuff[i] = ( DataBuff[i+1] + DataBuff[i-1] ) / 2 ;   // ȥ���������� ���ݸ���Ϊǰһ�κͺ�һ�ε�����ƽ��ֵ
        }
    }  
}


float MAGN_data_mean( float *DataBuff , int DataLen ) //ƽ��ֵ
{
    uint32_t dataNum = 0 ; 
    float data = 0 ; 
    float dataTemp = 0 ; 
    for (uint32_t i = 1 ; i <= DataLen ; i ++)
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



void MAGN_smooth_buff(float *DataBuff , float *data )
{
    float SmootData = 0 ; 
       
    MAGN_data_bubble(DataBuff , DATA_LEN) ; // ����
    
    MAGN_amplitude_smooth(DataBuff , DATA_LEN) ; // ��ֵ�˲�
   
    SmootData = MAGN_data_mean(DataBuff , DATA_LEN) ; // ƽ��ֵ
    
    *data = SmootData ; 
    //return SmootData ; // ����ƽ��ֵ

       
}



int data_err_pro(float *data1 , float *data2  )
{
    
    if  ( 500 <  fabs( *data1 - *data2 ) ) 
    {
        return 0 ; 
    }
    
    //if ( ( 2000 < fabs( *data1) ) || ( 2000 < fabs(*data2) ) )  {
       // return 0 ; 
   // }
    
    return 1 ;   
}

int MAGN_h_smooth()
{
  //  long double errVal = 500 ; 
    
    
    if ( 1  ) //��̬
    {
        MAGN_smooth_buff(HiDataBuff , &HiSmoothData) ; // hi�˲�(buff ���ֵ)
    }  
     
    if ( 1  ) //��̬
    {
        MAGN_smooth_buff(HnorDataBuff , &HnroSmoothData) ; // hnor�˲�
        if ( data_err_pro(&HiSmoothData , &HnroSmoothData ) ) // hi hnor���� )   // �����Ժ���
        {
            return 1 ; 
        }
        else
        {
            return 0 ;
        }
        
    }  
 
}



void MAGN_Quadrant_pro()
{
     // ������������Ƕ�ת��Ϊ0-360��
        
    if ( ( 0 > IMUDataBuff[MAGN_X] )  && ( 0 > IMUDataBuff[MAGN_Y] ) ) // ��һ,������ x- y-
    {
        if ( 0 < MAGNAngle) 
        {
            MAGNAngleTemp = MAGNAngle ;
        }
        else if ( 0 > MAGNAngle )
        {
            MAGNAngleTemp = 360 - ( MAGNAngle * (-1) ) ;
        }
    }
      
    if ( ( 0 < IMUDataBuff[MAGN_X] )  && ( 0 > IMUDataBuff[MAGN_Y] ) ) // �ڶ��������� x+ y-
    {
        MAGNAngleTemp = MAGNAngle + 180 ;
    }
        
    StepETmp =(uint32_t) MAGNAngleTemp ; 
    
    MAGN_write_buff(MAGNAngleBuff , StepETmp , DATA_LEN ) ; // д anglesun     
    // ���15�ȵ����ݱ��� ������ 5�ȵ�����д��buff 
  
}




int MAGN_angle_pro() // �شŴ���
{ 
     
    
    if ( MAGN_h_smooth() )//�Ƕ��˲� /// �˲�����Ҫ������   ���뷢����Ÿ��� ��Ĵ�ʩ  �����˲�ʧ��  ֱ��ȡ��gyro������  �˴δ�ǿ����ɾ��������
    {
         MAGN_Quadrant_pro() ; // ���޴���  
    }
    
    if ( MAGN_data_saturation(MAGNAngleBuff , DATA_LEN ) ) // �ش�buff����
    {
         MAGN_smooth_buff(MAGNAngleBuff , &MAGNSunAngleTmp) ; // hi�˲�(buff ���ֵ)
         MAGN_clear_buff( MAGNAngleBuff , DATA_LEN );  //������˲� ��Ҫ�����޸ĵ��ԡ�������������������������������
         //////////////////////////////////////////////////////////�����ʱ��Ƭ  ������ 
         MAGN_clear_buff(HiDataBuff ,DATA_LEN);
         MAGN_clear_buff(HnorDataBuff ,DATA_LEN);
         StepAngleProFlag = 1 ; //  
         return 0 ; 
    }
    return 1 ; 
    
}
