/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_wdog.c
 * @brief      ���Ź���������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-07-02
 */

#include "common.h"
#include "MKL_wdog.h"


static  uint32 wdog_cfg = 0;

/*!
 *  @brief      ��ʼ�����Ź�������ι��ʱ��
 *  @param      cfg     ι��ʱ��
 *  @since      v5.0
 */
void wdog_init(wdog_cfg_e cfg)
{

    wdog_cfg = cfg;
    SIM_COPC = wdog_cfg;
}


/*!
 *  @brief      ���ÿ��Ź�
 *  @since      v5.0
 */
void wdog_enable(void)
{
    SIM_COPC = wdog_cfg;
}


/*!
 *  @brief      ���ÿ��Ź�
 *  @since      v5.0
 */
void wdog_disable(void)
{
    wdog_cfg =  SIM_COPC;

    SIM_COPC &= ~SIM_COPC_COPT_MASK;
}

/*!
 *  @brief      ι��
 *  @since      v5.0
 */
void wdog_feed(void)
{
    SIM_SRVCOP = 0x55UL;
    SIM_SRVCOP = 0xAAUL;
}
