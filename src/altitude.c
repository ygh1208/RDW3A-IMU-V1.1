/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "imu.h"
#include <stdlib.h>
#include <math.h>
extern int32_t IMUDataBuff[] ; 


float InitPressureVal = 0 ; 
float AltitudeVal = 0 ; 
float PressureVal = 0 ; 
int tmp = 12555 ; 

void altitude_pro(){
      
    
    float AltitudeChang = 0 ; 
    float PressureChang = 0 ; 

    if ( 0 != IMUDataBuff[PRESSURE] ){
        PressureVal = (float)( IMUDataBuff[PRESSURE] * 0.08 ) ; 
        
        if ( 0 == InitPressureVal ){ // ³õÊ¼»¯ÆøÑ¹
        InitPressureVal = PressureVal ; 
        }
    
        if ( InitPressureVal < PressureVal ){
            PressureChang = (PressureVal - InitPressureVal) ;
        }
        else if ( InitPressureVal > PressureVal ){
            PressureChang = ( PressureVal - InitPressureVal ) ;
        }
    
        AltitudeVal = -1* PressureChang /0.11 ;
    
    }
    
  //PressureVal = PressureVal + 0.1 ;
    
  //  tmp ++ ; 
 //   PressureVal = tmp * 0.08 ; 
    
    
   
    

    /*
    if ( 0.001 < fabs(PressureChang) ){
        AltitudeVal += AltitudeChang ; 
        InitPressureVal =PressureVal ;
        AltitudeChang = 0 ; 
    }
    */
    
    
   // AltitudeChang = 0 ; 
    
}