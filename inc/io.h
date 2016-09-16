#ifndef __IO_H
#define __IO_H



#define LEDFLASH            3                                                   // led

#define LED_GPIO            LPC_GPIO0                                           // ledGPIO

#define LEDFlASH_OUTPUT     GPIO_SetDir(LED_GPIO, LEDFLASH, OUTPUT);            // led���
 
#define LEDFLASH_EN         GPIO_SetLowLevel(LED_GPIO, LEDFLASH, 1);            // ledʹ��

#define LEDFLASH_DIS        GPIO_SetHighLevel(LED_GPIO, LEDFLASH, 1);           // ledʧ��
   
                                               
#define KEY_POWER           5                                                   // ��Դ����

#define KEY_GPIO            LPC_GPIO0                                           // ��Դ����GPIO

#define KEY_POWER_INPUT     GPIO_SetDir(KEY_GPIO, KEY_POWER, INPUT)             // ��Դ��������

#define KEY_POWER_EN        0 == GPIO_GetPinValue(KEY_GPIO,KEY_POWER)           // ��Դ����ʹ��

#define KEY_POWER_DIS       1 == GPIO_GetPinValue(KEY_GPIO,KEY_POWER)           // ��Դ����ʧ��



#define POWER               6                                                   // ��Դ

#define POWER_GPIO          LPC_GPIO0                                           // ��ԴGPIO

#define POWER_OUTPUT        GPIO_SetDir(POWER_GPIO, POWER, OUTPUT);             // ��Դ���

#define POWER_EN            GPIO_SetHighLevel(POWER_GPIO, POWER, 1);            // ��Դʹ��

#define POWER_DIS           GPIO_SetLowLevel(POWER_GPIO, POWER, 1);             // ��Դʧ��






#endif /* __IO_H */