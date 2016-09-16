/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "imu.h"
#include <stdlib.h>
#include <math.h>
#include "time.h"
/* Private define ------------------------------------------------------------*/
#define ANGLE_ERR     15
/* Private functions ---------------------------------------------------------*/

/*�ⲿ����*/
extern float MAGNSunAngleTmp ; 
extern float GYROStepAngleTemp ; 

float PreStepAngle = 0 ; 
float SunAngle = 0 ; 

float MAGNPreStepAngle = 0 ; 
float MAGNSunAngle = 0 ;
float MAGNLastSunAngle = 0 ; 

float GYROPreStepAngle = 0 ; 
float GYROSunAngle = 0 ; 
float GYROLastSunAngle = 0 ; 

long double StepAngleErr = 0 ; // �����Ƕ����

void step_MAGN_angle_assign()
{
    MAGNSunAngle = MAGNSunAngleTmp ; // �شŽǶȸ�ֵ 
    MAGNPreStepAngle = MAGNSunAngle - MAGNLastSunAngle ; // ����شŵ���
    MAGNLastSunAngle = MAGNSunAngle ; 
}

void step_GYRO_angle_assign()
{
   // GYROSunAngle = GYROStepAngleTemp ; // ��ֵ����ǰ����
    GYROPreStepAngle = GYROSunAngle - GYROLastSunAngle ; // �ۼƽ����ʽǶ�
    GYROLastSunAngle = GYROSunAngle ; 
}

uint8_t step_err_pro()
{
    float tmp = 0 ; 
    tmp = fabs( GYROPreStepAngle - MAGNPreStepAngle ) ; // test
    if ( ANGLE_ERR < fabs( GYROPreStepAngle - MAGNPreStepAngle ) )
    {
        return 1;
        
    }
    
    return 0;
    
}

void step_angle_updata(uint8_t Err ) 
{
    if ( Err ) // �����
    {
        SunAngle = GYROSunAngle ;
        
        //PreStepAngle = GYROPreStepAngle ;
    }
    else  // �����
    {
        SunAngle = MAGNSunAngle ;
        GYROSunAngle = MAGNSunAngle ; 
        //PreStepAngle = ( MAGNPreStepAngle + GYROPreStepAngle ) / 2 ; 
    }  
}

void step_angle_pro()
{
    step_MAGN_angle_assign(); //�شŽǶȸ�ֵ
    
    step_GYRO_angle_assign() ; // �����ʽǶȸ�ֵ
    
    if ( step_err_pro() ) // ����
    {
        step_angle_updata(1) ; // �����
    }
    else 
    {
        step_angle_updata(0) ; // �����

    }
}