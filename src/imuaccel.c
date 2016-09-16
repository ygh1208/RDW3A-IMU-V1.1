/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "imu.h"
#include <stdlib.h>
#include <math.h>
#include "time.h"
/* Private define ------------------------------------------------------------*/
#define IMU_TIMER    0.005
#define ACCEL_WEIGHT   9.8
#define ACCEL_DIV_VAL  0.0033

/* Private functions ---------------------------------------------------------*/
/*�ⲿ����*/

extern int32_t IMUDataBuff[] ; 
// ���ٶ�ֵ��������

float AccelSetpM = 0 ;
float AccelStepCompM = 0;

//float ACCELDataBuff[510]= 0 ; // ���ٶ����ݻ���
int16_t WDBuffAdd = 0 ; // ���ٶ����ݻ����ַ
int16_t RDBuffAdd = 0 ; // ���ٶ����ݻ����ַ



float AccelMS = 0 ; // �ۼ� ��Ҫ�ֶ�����
float AccelM = 0 ;

float AccelMax = 0;
float AccelMin = 0;
float AccelK = 0;
float AccelS = 0;
uint32_t AccelTimer = 0 ;
    

// ��̬
int StaticTime = 0 ; 
int unStaticTime = 0 ; 

void accel_data_clear()
{
    AccelM = 0;
    AccelMS = 0;
    
    AccelMax = 0;
    AccelMin = 0;
    AccelK = 0;
    AccelS = 0;
    AccelTimer = 0;
    
}

float accel_off_weight()   // 
{
    float AccelAtmp = 0 ; 
    AccelAtmp=  ( IMUDataBuff[ACCEL_X] * IMUDataBuff[ACCEL_X] )  
              + ( IMUDataBuff[ACCEL_Y] * IMUDataBuff[ACCEL_Y] ) 
              + ( IMUDataBuff[ACCEL_Z] * IMUDataBuff[ACCEL_Z] ) ;
    AccelAtmp = sqrt(AccelAtmp) - 300; //  �������ȥ����
    
    
    // ������ٶ������ƽ����ֵ
    return AccelAtmp;
}

float accel_x(){
  
  float AccelAtmp = 0;
  AccelAtmp = IMUDataBuff[ACCEL_X] ;
  
  return AccelAtmp;
}

float accel_zero_smooth(float AccelA , float AccelW)
{
    // ���ٶ��˲�
    if ( ( 10 < AccelW ) || ( -10 > AccelW ) ) // ��̬�� ���ٶ�ƽ������MAXֵ��ﵽ11���� 
    { 
        //AccelR = AccelA ; 
        
        if ( 2000 > unStaticTime){
            unStaticTime ++ ;
        }
        StaticTime = 0 ;
        return AccelA;   // �жϲ����ڽ�ֹ״̬  ��� x��
        
    }
    else
    {
        //AccelR = 0 ; 
        if ( 2000 > StaticTime)
        {
            StaticTime ++ ; 
            unStaticTime = 0 ; 
        }
        
        return 0;
    }
}


float accel_speed(float AccelR , float AccelMS)
{
    float AccelG = 0 ;
    float AccelMS2 = 0 ;
    AccelG = AccelR * ACCEL_DIV_VAL ; // ��ֵת�����������ٶ�g  ��ʽ ��aֵ*3.3/1000 =g
    AccelMS2 = AccelG * ACCEL_WEIGHT ;  //�������ٶ�gת���� m/s2 ��ʽ��g*0.8=m/s2  
    if ( 0 == AccelR ) 
    {
        return 0 ;
    }
    else   
    {
        return ( AccelMS2 * IMU_TIMER ) + AccelMS  ; // ���ٶ�m/s2 ת�����ٶ�m/s ��ʽ��m/s2 *ʱ�� + ����= m/s
    }
  
}


void accel_distance( )
{
    if ( 0 == AccelMS )
    {
        AccelM = 0 ;
    }
    else
    {
        AccelM = ( AccelMS * IMU_TIMER ) + AccelM ; //�ٶ�m/s ת���ɾ���m    ��ʽ �� m/s*ʱ�� + ���� = M
    }
    
        
    AccelStepCompM = AccelM ;  // �ۼ�·�� ����·��
}



void accel_max_min_clear(){

  //  AccelRMax = 0;
  //  AccelRMaxTmp = 0;
   // AccelRMin = 0;
  //  AccelRMinTmp = 0;
}

void accel_max_scan(float Date){

  if (Date > AccelMax){
      
      AccelMax = Date ;
  } 
}

void accel_min_scan(float Date){

  if (Date < AccelMin){
      AccelMin = Date;
  }
}




void accel_timer_scan(){
     
    if (10000 > AccelTimer )
    {
        AccelTimer ++;
    }
  
    
}

    
    
void accel_data_pro() // ���ٶ����ݴ��� ʵʱ�� A
{
    uint8_t tmp = 0 ;   
    float AccelA = 0 ;
    float AccelW = 0;//ȥ������
    float AccelR = 0 ;
    
    AccelW = accel_off_weight() ; //ȥ�������ļ��ٶ�
    
    
    AccelA = accel_x();
    AccelR = accel_zero_smooth(AccelA,AccelW) ; 

    accel_max_scan(AccelR);
    accel_min_scan(AccelR);
    accel_timer_scan();
    
    /*
    
    ACCELDataBuff[WDBuffAdd] = AccelR ; //���ٶ����ݴ�����ٶ�BUFF
        
    if ( 500 > WDBuffAdd ) // д��ַ����
    {
        WDBuffAdd ++ ;
    }
    else 
    {
        WDBuffAdd = 0 ;
    }
    */
    
    
    
    AccelMS = accel_speed(AccelR , AccelMS); 
    
    accel_distance(AccelMS , AccelM ) ; 
  
   
}

