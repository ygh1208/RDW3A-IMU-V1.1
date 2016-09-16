/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "imu.h"
#include <stdlib.h>
#include <math.h>
#include "time.h"
/* Private define ------------------------------------------------------------*/
/*外部变量*/
extern float GYROSunAngle ;
extern uint32_t AccelTimer ;
//角速率变量定义

float GYROStepAngleData = 0 ; // 角速率计_单步角度值
//float GYROSunAngleData = 0 ; // 角速率累计角度值
//float GYROTmpData = 0 ; // 角速率临时数据
//float GYROAngleData = 0 ; // 角速率计_角度值

//long double GYROAngleBuff[10] = 0 ; 
float GYROStepAngleTemp = 0 ; 

int32_t IMUDataBuff[] = {
0x0000 , // gx
0x0000 , // gy
0x0000 , // gz
0x0000 , // ax
0x0000 , // ay 
0x0000 , // az
0x0000 , // mx
0x0000 , // my
0x0000 , // mz
0x0000 , // p
0x0000 , // pl
} ;


float GYROTmpData = 0 ;
  float GYROPoseAngleData = 0;
  
  
  
float GZDataBuff[510]= 0 ; // 加速度数据缓冲
int16_t GZBuffAdd = 0 ; // 加速度数据缓冲地址




void gyro_step_pose()
{

  GYROTmpData = IMUDataBuff[GYRO_Z] * 0.05 * IMU_TIMER ;
  
  if ( ( 50 > IMUDataBuff[GYRO_Z] ) && ( -50 < IMUDataBuff[GYRO_Z] ) )
  {
    GYROTmpData = 0 ; 
  }
  else
  {
    GYROPoseAngleData -= GYROTmpData ; // 角速率计_角度值
  }
  
  GZDataBuff[GZBuffAdd] = GYROPoseAngleData ; //加速度数据存入加速度BUFF
        
    if ( 500 > GZBuffAdd ) // 写地址递增
    {
        GZBuffAdd ++ ;
    }
    else 
    {
        GZBuffAdd = 0 ;
    }
  
}


uint32_t PoseFlag = 0;
uint32_t GyroSetpEffective = 0;// 单步角度识别

float GyroStepAngleCompareS = 0;
uint32_t GyroSetpUp = 0;
uint32_t GyroSetpDown = 0;


void gyro_step_distin_clear()
{
    GyroSetpDown = 0;
    GyroSetpUp = 0 ;
    GyroStepAngleCompareS = 0;
}

void gyro_step_data_clear()
{
    GYROPoseAngleData = 0;
}





void step_distin_err()
{
  
    uint8_t StepDelayTemp = 0; 
    if ( 100 < GYROPoseAngleData)
    {
         PoseFlag = 0;
    }
     
    StepDelayTemp = step_distin_delay(300) ;
    if (1 == StepDelayTemp ){
        PoseFlag = 0;
        
        AccelTimer = 0; //步伐超时，清空步伐时间
    }


}


uint32_t testx = 0;

void gyro_step_distin()  //////////////////待测试
{
  
    step_distin_err();
    
    
    /*
    
    switch(PoseFlag)
    {
    case 0 ://  5度 脚后跟
      if (0 == GyroSetpUp )//非上升
      {
          if( 5 < GYROPoseAngleData )
          {
              GyroSetpUp = 1;
          }
      }
      else// 上升
      {
          if(5 < GYROPoseAngleData )// 到达20度
          {
              PoseFlag =1 ;
              GyroSetpUp = 0;
              GyroStepAngleCompareS = 0;
              step_distin_delay(0);
          }
      }
      
      break;
      
    case 1 ://  至-10度 抬腿
      
      if(0 == GyroSetpDown) // 非下降
      {
          if( 0 > GYROPoseAngleData )
          {
              GyroSetpDown = 1;
          }
      }
      else // 下降
      {
          if(-10 > GYROPoseAngleData) //至-10度
          {
              PoseFlag =2;
              GyroSetpDown = 0;
              GyroStepAngleCompareS = 0;
          }
      }
      break;
      
      
    case 2 ://  至0度 踩踏
      
      if ( 3 < GYROPoseAngleData )
      {
          GyroSetpEffective = 1;
        
          PoseFlag = 0;
          step_distin_delay(0);
          
          Pacetestx ++;
          
      }
      
      break;
    
    }
    
    

    
    */
    
    switch(PoseFlag)
    {
    case 0 ://  5度 抬脚
      if (0 == GyroSetpUp )//非上升
      {
          if( 5 < GYROPoseAngleData )
          {
              GyroSetpUp = 1;
          }
      }
      else// 上升
      {
          if(10 < GYROPoseAngleData )// 到达20度
          {
              PoseFlag =1 ;
              GyroSetpUp = 0;
              GyroStepAngleCompareS = 0;
              step_distin_delay(0);
          }
      }
      
      break;
      
    case 1 ://  至-10度 抬腿
      
      if(0 == GyroSetpDown) // 非下降
      {
          if( 0 > GYROPoseAngleData )
          {
              GyroSetpDown = 1;
          }
      }
      else // 下降
      {
          if(3 > GYROPoseAngleData) //至-10度
          {
              GyroSetpEffective = 1;
        
              PoseFlag = 0;
              step_distin_delay(0);
          
              //Pacetestx ++;
          }
      }
      break;
      
    
    }
    
    
}

void gyro_step_data()
{
    gyro_step_pose();
    gyro_step_distin();// 识别
}



void gyro_data_pro() // 角速率数据处理 实时性A
{
    float GYROTmpData = 0 ;
    static float GYROAngleData = 0 ;
    GYROTmpData = IMUDataBuff[GYRO_Y] * 0.05 * IMU_TIMER ;
    if ( ( 50 > IMUDataBuff[GYRO_Y] ) && ( -50 < IMUDataBuff[GYRO_Y] ) )
    {
        GYROTmpData = 0 ; 
    }
    else
    {
        GYROSunAngle -= GYROTmpData ; // 角速率计_角度值
    }
    
    
    
    
    gyro_step_data();
    
}
