#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"

/*
 * Include �û��Զ����ͷ�ļ�
 */
#include  "MKL_BME.h"           //λ����
#include  "MKL_wdog.h"          //���Ź�
#include  "MKL_gpio.h"          //IO�ڲ���
#include  "MKL_uart.h"          //����
#include  "MKL_SysTick.h"       //�δ�ʱ��
#include  "MKL_lptmr.h"         //�͹��Ķ�ʱ��(��ʱ�������������ʱ����ʱ)
#include  "MKL_i2c.h"           //I2C
#include  "MKL_spi.h"           //SPI
#include  "MKL_tpm.h"           //TPM������K60�� FTM ��pwm�����������
#include  "MKL_pit.h"           //PIT
#include  "MKL_adc.h"           //ADC
#include  "MKL_dac.h"           //DAC
#include  "MKL_dma.h"           //DMA
#include  "MKL_FLASH.h"         //FLASH


#include  "VCAN_LED.H"          //LED
#include  "VCAN_KEY.H"          //KEY
#include  "VCAN_MMA7455.h"      //������ٶ�MMA7455
#include  "VCAN_NRF24L0.h"      //����ģ��NRF24L01+

#include  "VCAN_camera.h"       //����ͷ��ͷ�ļ�
#include  "VCAN_LCD.h"          //Һ����ͷ�ļ�

#include  "VCAN_TSL1401.h"      //����CCD
#include  "VCAN_key_event.h"    //������Ϣ����
#include  "VCAN_NRF24L0_MSG.h"  //����ģ����Ϣ����



#include  "LandzoPredator.h"
#include  "Get_Black_Center_Line.h"
#include  "key&Oled.h"
#include "isr.h"
#include  "LandzoOLED.h"
#include "Speed_Ctrl.h"

#endif  //__INCLUDE_H__
