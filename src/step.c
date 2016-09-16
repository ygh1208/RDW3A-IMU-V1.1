/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "imu.h"
#include <stdlib.h>
#include <math.h>
#include "time.h"
/* Private define ------------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

uint32_t StepAngletmp = 0 ; 
extern int32_t IMUDataBuff[] ; 
extern uint32_t GyroSetpEffective;
extern uint32_t AccelTimer;
extern int32_t Pacetestx;



int32_t testK = 0;
int32_t testS = 0;
int32_t Pacetestx = 0;

uint8_t buffer1[]={
"0" "1" "2" "3" "4" "5" "6" "7" "8" "9"
};



uint8_t IMUDataBuffer[] = { " # +00000000 +00000000 +00000 000%" } ; 
                           //012345678901234567890123456789
                           //0         1         2        
                           //   E         N         H 




extern float MAGNStepAngle ; // 地磁单步角度值
extern float MAGNSunAngle ; 

extern int StaticTime  ; 

extern int unStaticTime ; 

extern uint8_t UartSendLabel ; 

extern uint8_t GYROStepProFlag ; 
extern uint32_t GyroSetpEffective ;// 单步角度识别

extern uint8_t MAGNAngleProLabel ; 
extern uint8_t StepAngleProFlag ;


long double StepE = 0 ; 
long double StepN = 0 ; 
int32_t StepETmp = 0 ; 
int32_t StepNTmp = 0 ; 

uint8_t StaticLabel = 0 ; 
extern float AccelSetpM ;

extern float SunAngle ; 
extern float AltitudeVal ; 
extern float AccelStepCompM;

extern float GYROPoseAngleData;

extern float AccelK ;
extern float AccelMax ;
extern float AccelMin ;
extern float AccelK ;
extern float AccelS ;


float StepAngleValue = 0 ; // 单步角度值
float AngleValue = 0 ; // 角度值

int32_t AltitudeValTmp = 0;








float K_VOL[] = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9  };

float S_Vol[] = { 2 , 2.5 , 3 , 3.5 , 4 , 4.5 , 5 , 5.5 , 6 , 6.5 , 7 , 7.5 , 8 , 8.5 , 9 , 9.5 ,10 , 10.5 , 11 , 11.5  };



float K_err = 0.5;
int32_t PaceRecognitionFlag = 0;//步伐识别

void step_m_pro()
{
 
    AccelSetpM += AccelStepCompM * cosl(GYROPoseAngleData * 3.14159/180 );
}



void step_coord_pro() // 单步坐标处理
{
    /*
    test 0
    直线标定测试
  把角度设置为0  查看x直线距离
  
  */
     //test 0
    float SunAngleTEST = 0;
    StepE = StepE + AccelSetpM * sin( SunAngleTEST * 3.14159/180 ) ;   
    StepN = StepN + AccelSetpM * cos( SunAngleTEST * 3.14159/180 ) ; ; 
  
  
   // StepE = StepE + AccelSetpM * sin( SunAngle * 3.14159/180 ) ; 
   // StepN = StepN + AccelSetpM * cos( SunAngle * 3.14159/180 ) ; 
        
   
  
    StepETmp = StepE  ; 
    StepNTmp = StepN  ; 
       
    if ( 0 < StepE )
    { 
        IMUDataBuffer[3] = '+' ;  
        IMUDataBuffer[4] = buffer1[(StepETmp%100000000/10000000)];
        IMUDataBuffer[5] = buffer1[(StepETmp%100000000%10000000/1000000)]; 
        IMUDataBuffer[6] = buffer1[(StepETmp%100000000%10000000%1000000/100000)];
        IMUDataBuffer[7] = buffer1[(StepETmp%100000000%10000000%1000000%100000/10000)];
        IMUDataBuffer[8] = buffer1[(StepETmp%100000000%10000000%1000000%100000%10000/1000)];
        IMUDataBuffer[9] = buffer1[(StepETmp%100000000%10000000%1000000%100000%10000%1000/100)];
        IMUDataBuffer[10] = buffer1[(StepETmp%100000000%10000000%1000000%100000%10000%1000%100/10)];
        IMUDataBuffer[11] = buffer1[(StepETmp%100000000%10000000%1000000%100000%10000%1000%100%10)];
    }
    else if ( 0 > StepE ) 
    {
        StepETmp = StepETmp * (-1) ;
        IMUDataBuffer[3] = '-' ;   
        IMUDataBuffer[4] = buffer1[(StepETmp%100000000/10000000)];
        IMUDataBuffer[5] = buffer1[(StepETmp%100000000%10000000/1000000)]; 
        IMUDataBuffer[6] = buffer1[(StepETmp%100000000%10000000%1000000/100000)];
        IMUDataBuffer[7] = buffer1[(StepETmp%100000000%10000000%1000000%100000/10000)];
        IMUDataBuffer[8] = buffer1[(StepETmp%100000000%10000000%1000000%100000%10000/1000)];
        IMUDataBuffer[9] = buffer1[(StepETmp%100000000%10000000%1000000%100000%10000%1000/100)];
        IMUDataBuffer[10] = buffer1[(StepETmp%100000000%10000000%1000000%100000%10000%1000%100/10)];
        IMUDataBuffer[11] = buffer1[(StepETmp%100000000%10000000%1000000%100000%10000%1000%100%10)];
    }
        
    if ( 0 < StepN )
    { 
        IMUDataBuffer[13] = '+' ;  
        IMUDataBuffer[14] = buffer1[(StepNTmp%100000000/10000000)];
        IMUDataBuffer[15] = buffer1[(StepNTmp%100000000%10000000/1000000)]; 
        IMUDataBuffer[16] = buffer1[(StepNTmp%100000000%10000000%1000000/100000)];
        IMUDataBuffer[17] = buffer1[(StepNTmp%100000000%10000000%1000000%100000/10000)];
        IMUDataBuffer[18] = buffer1[(StepNTmp%100000000%10000000%1000000%100000%10000/1000)];
        IMUDataBuffer[19] = buffer1[(StepNTmp%100000000%10000000%1000000%100000%10000%1000/100)];
        IMUDataBuffer[20] = buffer1[(StepNTmp%100000000%10000000%1000000%100000%10000%1000%100/10)];
        IMUDataBuffer[21] = buffer1[(StepNTmp%100000000%10000000%1000000%100000%10000%1000%100%10)];
    }
    else if ( 0 > StepN ) 
    {
        StepNTmp = StepNTmp * (-1) ;
        IMUDataBuffer[13] = '-' ;  
        IMUDataBuffer[14] = buffer1[(StepNTmp%100000000/10000000)];
        IMUDataBuffer[15] = buffer1[(StepNTmp%100000000%10000000/1000000)]; 
        IMUDataBuffer[16] = buffer1[(StepNTmp%100000000%10000000%1000000/100000)];
        IMUDataBuffer[17] = buffer1[(StepNTmp%100000000%10000000%1000000%100000/10000)];
        IMUDataBuffer[18] = buffer1[(StepNTmp%100000000%10000000%1000000%100000%10000/1000)];
        IMUDataBuffer[19] = buffer1[(StepNTmp%100000000%10000000%1000000%100000%10000%1000/100)];
        IMUDataBuffer[20] = buffer1[(StepNTmp%100000000%10000000%1000000%100000%10000%1000%100/10)];
        IMUDataBuffer[21] = buffer1[(StepNTmp%100000000%10000000%1000000%100000%10000%1000%100%10)];
    } 
    

   
    AltitudeValTmp =  (int)AltitudeVal ;
    
    if ( 0 <= AltitudeValTmp ){
    
        IMUDataBuffer[23] = '+';
        IMUDataBuffer[24] = buffer1[(AltitudeValTmp%100000/10000)]; // 高度
        IMUDataBuffer[25] = buffer1[(AltitudeValTmp%100000%10000/1000)];
        IMUDataBuffer[26] = buffer1[(AltitudeValTmp%100000%10000%1000/100)];
        IMUDataBuffer[27] = buffer1[(AltitudeValTmp%100000%10000%1000%100/10)];
        IMUDataBuffer[28] = buffer1[(AltitudeValTmp%100000%10000%1000%100%10)];
    }
    else{
    
        AltitudeValTmp = AltitudeValTmp * (-1);
        IMUDataBuffer[23] = '-';
        IMUDataBuffer[24] = buffer1[(AltitudeValTmp%100000/10000)]; // 高度
        IMUDataBuffer[25] = buffer1[(AltitudeValTmp%100000%10000/1000)];
        IMUDataBuffer[26] = buffer1[(AltitudeValTmp%100000%10000%1000/100)];
        IMUDataBuffer[27] = buffer1[(AltitudeValTmp%100000%10000%1000%100/10)];
        IMUDataBuffer[28] = buffer1[(AltitudeValTmp%100000%10000%1000%100%10)];
    }
    

    
    StepAngletmp = SunAngle ; 
    StepAngletmp = Pacetestx;// test 测试步数用
    //StepAngletmp = testK;// test 测试   k
    //StepAngletmp = testS;// test 测试s
    IMUDataBuffer[30] = buffer1[(StepAngletmp/100)];// 角度
    IMUDataBuffer[31] = buffer1[(StepAngletmp%100/10)];
    IMUDataBuffer[32] = buffer1[(StepAngletmp%100%10)];
    
    AccelSetpM = 0 ; 
    UartSendLabel = 1 ;
    
}



void K_pro(){

    /////////////////////k 的处理公式。。。。。。。。。。。。。。。。。。。。。。。。。。。。
    float AccelMMerr = 0;
    AccelMMerr = AccelMax - AccelMin ;
    if ( 200 > AccelMMerr )
    {
        AccelK = K_VOL[18] * K_err;
        testK = 1;// test  
    }
    else if( 300 > AccelMMerr ){
        AccelK = K_VOL[17] * K_err;
        testK = 2;// test  
    }
    else if( 400 > AccelMMerr ){
        AccelK = K_VOL[16] * K_err;
        testK = 3;// test  
    }
    else if( 500 > AccelMMerr ){
        AccelK = K_VOL[15] * K_err;
        testK = 4;// test  
    }
    else if( 600 > AccelMMerr ){
        AccelK = K_VOL[14] * K_err;
        testK = 5;// test  
    }
    else if( 700 > AccelMMerr ){
        AccelK = K_VOL[13] * K_err;
        testK = 6;// test  
    }
    else if( 800 > AccelMMerr ){
        AccelK = K_VOL[12] * K_err;
        testK = 7;// test  
    }
    else if( 900 > AccelMMerr ){
        AccelK = K_VOL[11] * K_err;
        testK = 8;// test  
    }
    else if( 1000 > AccelMMerr ){
        AccelK = K_VOL[10] * K_err;
        testK = 9;// test  
    }
    else if( 1100 > AccelMMerr ){
        AccelK = K_VOL[9] * K_err;
        testK = 10;// test  
    }
    else if( 1200 > AccelMMerr ){
        AccelK = K_VOL[8] * K_err;
        testK = 11;// test  
    }
    else if( 1300 > AccelMMerr ){
        AccelK = K_VOL[7] * K_err;
        testK = 12;// test  
    }
    else if( 1400 > AccelMMerr ){
        AccelK = K_VOL[6] * K_err;
        testK = 13;// test  
    }
    else if( 1500 > AccelMMerr ){
        AccelK = K_VOL[5] * K_err;
        testK = 14;// test  
    }
    else if( 1600 > AccelMMerr ){
        AccelK = K_VOL[4]* K_err;
        testK = 15;// test  
    }
    else if( 1700 > AccelMMerr ){
        AccelK = K_VOL[3]* K_err;
        testK = 16;// test  
    }
    else if( 1800 > AccelMMerr ){
        AccelK = K_VOL[2]* K_err;
        testK = 17;// test  
    }
    else if( 1900 > AccelMMerr ){
        AccelK = K_VOL[1]* K_err;
        testK = 18;// test  
    }
    else if( 2000 > AccelMMerr ){
        AccelK = K_VOL[0]* K_err;
        testK = 19;// test  
    }
    
      
      
    
    //AccelSetpM *= AccelK;
}


void S_pro(){

    static float AccelMMerr = 0;
    static float AccelSTmp = 0;
    AccelMMerr = AccelMax - AccelMin ;
    AccelSTmp = AccelMMerr * AccelK;
    if ( 150 > AccelSTmp ){
        AccelS = S_Vol[0] ;
        testS = 1;//test
    }
    else if( 170 > AccelSTmp  ){
        AccelS = S_Vol[1] ;
        testS = 2;//test
    }
    else if( 190 > AccelSTmp  ){
        AccelS = S_Vol[2] ;
        testS = 3;//test
    }
    else if( 210 > AccelSTmp  ){
        AccelS = S_Vol[3] ;
        testS = 4;//test
    }
    else if( 230 > AccelSTmp  ){
        AccelS = S_Vol[4] ;
        testS = 5;//test
    }
    else if( 250 > AccelSTmp  ){
        AccelS = S_Vol[5] ;
        testS = 6;//test
    }
    else if( 270 > AccelSTmp  ){
        AccelS = S_Vol[6] ;
        testS = 7;//test
    }
    else if( 290 > AccelSTmp  ){
        AccelS = S_Vol[7] ;
        testS = 8;//test
    }
    else if( 310 > AccelSTmp  ){
        AccelS = S_Vol[8] ;
        testS = 9;//test
    }
    else if( 330 > AccelSTmp  ){
        AccelS = S_Vol[9] ;
        testS = 10;//test
    }
    else if( 350> AccelSTmp  ){
        AccelS = S_Vol[10] ;
        testS = 11;//test
    }
    else if( 370 > AccelSTmp  ){
        AccelS = S_Vol[11] ;
        testS = 12;//test
    }
    else if( 390 > AccelSTmp  ){
        AccelS = S_Vol[12] ;
        testS = 13;//test
    }
    else if ( 410 > AccelSTmp  ){
        AccelS = S_Vol[13] ;
        testS = 14;//test
    }
    else if( 430 > AccelSTmp  ){
        AccelS = S_Vol[14] ;
        testS = 15;//test
    }
    else if( 450 > AccelSTmp  ){
        AccelS = S_Vol[15] ;
        testS = 16;//test
    }
    else if( 470 > AccelSTmp  ){
        AccelS = S_Vol[16] ;
        testS = 17;//test
    }
    else if( 490 > AccelSTmp  ){
        AccelS = S_Vol[17] ;
        testS = 18;//test
    }
    else if( 510 > AccelSTmp  ){
        AccelS = S_Vol[18] ;
        testS = 19;//test
    }
    else if( 530 > AccelSTmp  ){
        AccelS = S_Vol[19] ;
        testS = 20;//test
    }
    
}

float xxx = 0;
void M_pro(){

    
    //AccelS = 5.3; //test
    AccelStepCompM = AccelS * 1000 * AccelTimer * 0.005 / 3600;  //  乘第一个1000  单位公里km 变成m   
                                         //第一个时间 * time   第二个时间  是 采样值0.005
    
    
    
}

void static_state_pro() // 静止状态 处理
{
 
    if ( ( 6 < unStaticTime ) && ( 0 == StaticTime ) ){
        StaticLabel = 0 ;
    }
     
        incline_pro(); // 角度处理
    if (incline_scan(15,-15)) //角度平行
    {
        if ( 2 < StaticTime )
        {  
            StaticLabel = 1 ; // 置位，静止标志处于静止状态 
                
        }    
        
        
        if( 1 == GyroSetpEffective )
        {

            K_pro();
            S_pro();
                
        M_pro();
        step_m_pro();
                
        gyro_step_distin_clear();
        accel_data_clear();
        GyroSetpEffective = 0;
        PaceRecognitionFlag = 1;

        
        Pacetestx ++;   
        
        }
    
    
    
    }
    
    
    if(incline_scan(2,-2))// 角度平行
    {
        gyro_step_data_clear();
        
    }
    
    
     
      
    
    
    
}



/*

void angle_smooth() 
{
    
    MAGN_angle_smooth() ; 
    
  
    
  
    
}
*/