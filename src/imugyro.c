/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "imu.h"
#include <stdlib.h>
#include <math.h>
#include "time.h"
/* Private define ------------------------------------------------------------*/
/*�ⲿ����*/
extern float GYROSunAngle ;
extern uint32_t AccelTimer ;
//�����ʱ�������

float GYROStepAngleData = 0 ; // �����ʼ�_�����Ƕ�ֵ
//float GYROSunAngleData = 0 ; // �������ۼƽǶ�ֵ
//float GYROTmpData = 0 ; // ��������ʱ����
//float GYROAngleData = 0 ; // �����ʼ�_�Ƕ�ֵ

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
  
  
  
float GZDataBuff[510]= 0 ; // ���ٶ����ݻ���
int16_t GZBuffAdd = 0 ; // ���ٶ����ݻ����ַ




void gyro_step_pose()
{

  GYROTmpData = IMUDataBuff[GYRO_Z] * 0.05 * IMU_TIMER ;
  
  if ( ( 50 > IMUDataBuff[GYRO_Z] ) && ( -50 < IMUDataBuff[GYRO_Z] ) )
  {
    GYROTmpData = 0 ; 
  }
  else
  {
    GYROPoseAngleData -= GYROTmpData ; // �����ʼ�_�Ƕ�ֵ
  }
  
  GZDataBuff[GZBuffAdd] = GYROPoseAngleData ; //���ٶ����ݴ�����ٶ�BUFF
        
    if ( 500 > GZBuffAdd ) // д��ַ����
    {
        GZBuffAdd ++ ;
    }
    else 
    {
        GZBuffAdd = 0 ;
    }
  
}


uint32_t PoseFlag = 0;
uint32_t GyroSetpEffective = 0;// �����Ƕ�ʶ��

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
        
        AccelTimer = 0; //������ʱ����ղ���ʱ��
    }


}


uint32_t testx = 0;

void gyro_step_distin()  //////////////////������
{
  
    step_distin_err();
    
    
    /*
    
    switch(PoseFlag)
    {
    case 0 ://  5�� �ź��
      if (0 == GyroSetpUp )//������
      {
          if( 5 < GYROPoseAngleData )
          {
              GyroSetpUp = 1;
          }
      }
      else// ����
      {
          if(5 < GYROPoseAngleData )// ����20��
          {
              PoseFlag =1 ;
              GyroSetpUp = 0;
              GyroStepAngleCompareS = 0;
              step_distin_delay(0);
          }
      }
      
      break;
      
    case 1 ://  ��-10�� ̧��
      
      if(0 == GyroSetpDown) // ���½�
      {
          if( 0 > GYROPoseAngleData )
          {
              GyroSetpDown = 1;
          }
      }
      else // �½�
      {
          if(-10 > GYROPoseAngleData) //��-10��
          {
              PoseFlag =2;
              GyroSetpDown = 0;
              GyroStepAngleCompareS = 0;
          }
      }
      break;
      
      
    case 2 ://  ��0�� ��̤
      
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
    case 0 ://  5�� ̧��
      if (0 == GyroSetpUp )//������
      {
          if( 5 < GYROPoseAngleData )
          {
              GyroSetpUp = 1;
          }
      }
      else// ����
      {
          if(10 < GYROPoseAngleData )// ����20��
          {
              PoseFlag =1 ;
              GyroSetpUp = 0;
              GyroStepAngleCompareS = 0;
              step_distin_delay(0);
          }
      }
      
      break;
      
    case 1 ://  ��-10�� ̧��
      
      if(0 == GyroSetpDown) // ���½�
      {
          if( 0 > GYROPoseAngleData )
          {
              GyroSetpDown = 1;
          }
      }
      else // �½�
      {
          if(3 > GYROPoseAngleData) //��-10��
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
    gyro_step_distin();// ʶ��
}



void gyro_data_pro() // ���������ݴ��� ʵʱ��A
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
        GYROSunAngle -= GYROTmpData ; // �����ʼ�_�Ƕ�ֵ
    }
    
    
    
    
    gyro_step_data();
    
}
