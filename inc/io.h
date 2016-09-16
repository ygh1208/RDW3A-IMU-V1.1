#ifndef __IO_H
#define __IO_H



#define LEDFLASH            3                                                   // led

#define LED_GPIO            LPC_GPIO0                                           // ledGPIO

#define LEDFlASH_OUTPUT     GPIO_SetDir(LED_GPIO, LEDFLASH, OUTPUT);            // led输出
 
#define LEDFLASH_EN         GPIO_SetLowLevel(LED_GPIO, LEDFLASH, 1);            // led使能

#define LEDFLASH_DIS        GPIO_SetHighLevel(LED_GPIO, LEDFLASH, 1);           // led失能
   
                                               
#define KEY_POWER           5                                                   // 电源按键

#define KEY_GPIO            LPC_GPIO0                                           // 电源按键GPIO

#define KEY_POWER_INPUT     GPIO_SetDir(KEY_GPIO, KEY_POWER, INPUT)             // 电源按键输入

#define KEY_POWER_EN        0 == GPIO_GetPinValue(KEY_GPIO,KEY_POWER)           // 电源按键使能

#define KEY_POWER_DIS       1 == GPIO_GetPinValue(KEY_GPIO,KEY_POWER)           // 电源按键失能



#define POWER               6                                                   // 电源

#define POWER_GPIO          LPC_GPIO0                                           // 电源GPIO

#define POWER_OUTPUT        GPIO_SetDir(POWER_GPIO, POWER, OUTPUT);             // 电源输出

#define POWER_EN            GPIO_SetHighLevel(POWER_GPIO, POWER, 1);            // 电源使能

#define POWER_DIS           GPIO_SetLowLevel(POWER_GPIO, POWER, 1);             // 电源失能






#endif /* __IO_H */