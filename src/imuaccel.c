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
/*外部变量*/

extern int32_t IMUDataBuff[] ; 
// 加速度值变量定义

float AccelSetpM = 0 ;
float AccelStepCompM = 0;

//float ACCELDataBuff[510]= 0 ; // 加速度数据缓冲
int16_t WDBuffAdd = 0 ; // 加速度数据缓冲地址
int16_t RDBuffAdd = 0 ; // 加速度数据缓冲地址



float AccelMS = 0 ; // 累计 需要手动清零
float AccelM = 0 ;

float AccelMax = 0;
float AccelMin = 0;
float AccelK = 0;
float AccelS = 0;
uint32_t AccelTimer = 0 ;
    

// 静态
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
    AccelAtmp = sqrt(AccelAtmp) - 300; //  将结果减去重力
    
    
    // 计算加速度三轴的平方根值
    return AccelAtmp;
}

float accel_x(){
  
  float AccelAtmp = 0;
  AccelAtmp = IMUDataBuff[ACCEL_X] ;
  
  return AccelAtmp;
}

float accel_zero_smooth(float AccelA , float AccelW)
{
    // 加速度滤波
    if ( ( 10 < AccelW ) || ( -10 > AccelW ) ) // 静态是 加速度平方根的MAX值会达到11左右 
    { 
        //AccelR = AccelA ; 
        
        if ( 2000 > unStaticTime){
            unStaticTime ++ ;
        }
        StaticTime = 0 ;
        return AccelA;   // 判断不处于禁止状态  输出 x轴
        
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
    AccelG = AccelR * ACCEL_DIV_VAL ; // 数值转化成重力加速度g  公式 ：a值*3.3/1000 =g
    AccelMS2 = AccelG * ACCEL_WEIGHT ;  //重力加速度g转换成 m/s2 公式：g*0.8=m/s2  
    if ( 0 == AccelR ) 
    {
        return 0 ;
    }
    else   
    {
        return ( AccelMS2 * IMU_TIMER ) + AccelMS  ; // 加速度m/s2 转换成速度m/s 公式：m/s2 *时间 + 积分= m/s
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
        AccelM = ( AccelMS * IMU_TIMER ) + AccelM ; //速度m/s 转换成距离m    公式 ： m/s*时间 + 积分 = M
    }
    
        
    AccelStepCompM = AccelM ;  // 累加路程 单步路程
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

    
    
void accel_data_pro() // 加速度数据处理 实时性 A
{
    uint8_t tmp = 0 ;   
    float AccelA = 0 ;
    float AccelW = 0;//去除重力
    float AccelR = 0 ;
    
    AccelW = accel_off_weight() ; //去除重力的加速度
    
    
    AccelA = accel_x();
    AccelR = accel_zero_smooth(AccelA,AccelW) ; 

    accel_max_scan(AccelR);
    accel_min_scan(AccelR);
    accel_timer_scan();
    
    /*
    
    ACCELDataBuff[WDBuffAdd] = AccelR ; //加速度数据存入加速度BUFF
        
    if ( 500 > WDBuffAdd ) // 写地址递增
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

