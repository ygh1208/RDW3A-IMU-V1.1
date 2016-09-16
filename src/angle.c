/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "imu.h"
#include <stdlib.h>
#include <math.h>
#include "time.h"
/* Private define ------------------------------------------------------------*/
#define ANGLE_ERR     15
/* Private functions ---------------------------------------------------------*/

/*外部变量*/
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

long double StepAngleErr = 0 ; // 单步角度误差

void step_MAGN_angle_assign()
{
    MAGNSunAngle = MAGNSunAngleTmp ; // 地磁角度赋值 
    MAGNPreStepAngle = MAGNSunAngle - MAGNLastSunAngle ; // 计算地磁单步
    MAGNLastSunAngle = MAGNSunAngle ; 
}

void step_GYRO_angle_assign()
{
   // GYROSunAngle = GYROStepAngleTemp ; // 赋值给当前单步
    GYROPreStepAngle = GYROSunAngle - GYROLastSunAngle ; // 累计角速率角度
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
    if ( Err ) // 有误差
    {
        SunAngle = GYROSunAngle ;
        
        //PreStepAngle = GYROPreStepAngle ;
    }
    else  // 无误差
    {
        SunAngle = MAGNSunAngle ;
        GYROSunAngle = MAGNSunAngle ; 
        //PreStepAngle = ( MAGNPreStepAngle + GYROPreStepAngle ) / 2 ; 
    }  
}

void step_angle_pro()
{
    step_MAGN_angle_assign(); //地磁角度赋值
    
    step_GYRO_angle_assign() ; // 角速率角度赋值
    
    if ( step_err_pro() ) // 误差处理
    {
        step_angle_updata(1) ; // 有误差
    }
    else 
    {
        step_angle_updata(0) ; // 无误差

    }
}