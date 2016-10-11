/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       VCAN_MK60_conf.h
 * @brief      ɽ��K60 ƽ̨�����ļ�
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-06-26
 */

#ifndef __VCAN_MK60_CONF_H__
#define __VCAN_MK60_CONF_H__

#include    "MKL_mcg.h"

#define LCD_ST7735S     2       //LCD_ST7735S �� LCD_ST7735R ���һ�����Ĵ����������в�ͬ
#define LCD_ST7735R     3       //
#define USE_LCD         LCD_ST7735R             //ѡ��ʹ�õ� LCD

/*
 * ѡ���Ƿ����������Ϣ���������ע������ĺ궨��
 */
#define DEBUG_PRINT

/*
 * ���徧��ʱ�ӣ���λΪMHz
 */
#define EXTAL_IN_MHz            (8)

/*
 * ���� PLL ��Ƶ Ƶ��
 */
#define PLL_CLK                PLL200      // �� PLL_e ��ѡ�� ���ṩ�� ���÷���
                                          // core/bus Ƶ�� ���� pll ��������Ƶ����

#define MAX_CORE_CLK            100        // core     (bus        >= pll/16  )
#define MAX_BUS_CLK             40         // bus      (bus        >= core/16  )



/*********************   �Զ��� ʱ��Ƶ�� ��Ƶ����   ********************/
//��� CORE_CLK Ϊ PLLUSR ����Ϊ�Զ���ģʽ ������������Ч
#define PRDIV             10        // MCG_CLK_MHZ = EXTAL_IN_MHz/(PRDIV+1)*(VDIV+24)
#define VDIV              29
#define CORE_DIV          0         //  core = mcg/ ( CORE_DIV  + 1 )
#define BUS_DIV           1         //  bus  = mcg/ ( BUS_DIV   + 1 )



/*
 * ���� printf���� �� ��������˿� �� ������Ϣ
 */
#define VCAN_PORT           UART0
#define VCAN_BAUD           115200

/*
 * ������ʱ����
 */
#if  0
#include "MKL_lptmr.h"
#define     DELAY()         lptmr_delay_ms(500)
#define     DELAY_MS(ms)    lptmr_delay_ms(ms)
#define     DELAY_US(us)    lptmr_delay_us(us)
#elif   0
#include "MKL_pit.h"
#define DELAY()         pit_delay_ms(PIT1,500)
#define DELAY_MS(ms)    pit_delay_ms(PIT1,ms)
#define DELAY_US(us)    pit_delay_us(PIT1,us)
#else
#include "MKL_SysTick.h"
#define DELAY()         systick_delay_ms(500)
#define DELAY_MS(ms)    systick_delay_ms(ms)
#define DELAY_US(us)    systick_delay_us(us)
#endif


/*
 * ���ö��Ժ���ʵ�ֺ���
 */
void assert_failed(char *, int);

#if ( defined( DEBUG ) && defined( DEBUG_PRINT ))
#define ASSERT(expr) \
    if (!(expr)) \
        assert_failed(__FILE__, __LINE__)
#else
#define ASSERT(expr)
#endif

/*
 * ���õ����������
 */
#if( defined(DEBUG) && defined(DEBUG_PRINT))
#define DEBUG_PRINTF(FORMAT,...)        do{printf(FORMAT,##__VA_ARGS__);}while(0)	/*�����ӡ������Ϣʱ���뽫������ע�͵�*/
#else
#define DEBUG_PRINTF(FORMAT,...)
#endif

/*
 * ����ǰ��⣬��ֹ �������ؿ�
 */
void start_check();
#ifdef DEBUG
#define SRART_CHECK()       start_check()
#else
#define SRART_CHECK()
#endif


#endif /* __VCAN_MK60_CONF_H__  */
