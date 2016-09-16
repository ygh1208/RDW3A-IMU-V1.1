/* Includes ------------------------------------------------------------------*/  
#include "lpc12xx_libcfg.h"
#include "head.h"
#include <stdlib.h>
#include <math.h>
/* Private define ------------------------------------------------------------*/

uint8_t StepDistinDelaySw = FLAG_FALSE;
uint32_t StepDistinDelayTimer = 0;


uint8_t step_distin_delay(uint32_t DelayValue){

    if ( 0 == DelayValue )
    {
        StepDistinDelaySw = FLAG_FALSE ;
	StepDistinDelayTimer = 0 ;
        return FLAG_FALSE ;
    }
    
    if ( FLAG_FALSE == StepDistinDelaySw )
    {
        StepDistinDelaySw = FLAG_TURE ;
    }
	      
    if ( StepDistinDelayTimer < DelayValue )
    {
        return FLAG_FALSE ;
    }
	  	  
    if ( StepDistinDelayTimer > DelayValue )
    {
        StepDistinDelaySw = FLAG_FALSE ;
	StepDistinDelayTimer = 0 ;
	return FLAG_TURE ;
    }   
}