/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MKL26_wdog.h
 * @brief      ���Ź���������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-07-02
 */

#ifndef __MKL26_WDOG_H__
#define __MKL26_WDOG_H__

/********************************************************************/

typedef enum
{
    WDOG_LPO_32MS   = SIM_COPC_COPT(1) ,                            // ʹ�� LPO 1KHz ʱ�ӣ�ι��ʱ��Ϊ32ms
    WDOG_LPO_256MS  = SIM_COPC_COPT(2) ,                            // ʹ�� LPO 1KHz ʱ�ӣ�ι��ʱ��Ϊ256ms
    WDOG_LPO_1024MS = SIM_COPC_COPT(3) ,                            // ʹ�� LPO 1KHz ʱ�ӣ�ι��ʱ��Ϊ1024ms

    WDOG_BUS_8192T  = SIM_COPC_COPT(1) | SIM_COPC_COPCLKS_MASK,     // ʹ�� BUS ʱ�ӣ�ι��ʱ��Ϊ8192T ��T Ϊbusʱ������
    WDOG_BUS_65536T = SIM_COPC_COPT(2) | SIM_COPC_COPCLKS_MASK,     // ʹ�� BUS ʱ�ӣ�ι��ʱ��Ϊ65536T ��T Ϊbusʱ������
    WDOG_BUS_262144T= SIM_COPC_COPT(3) | SIM_COPC_COPCLKS_MASK,     // ʹ�� BUS ʱ�ӣ�ι��ʱ��Ϊ262144T ��T Ϊbusʱ������
}wdog_cfg_e;



extern void wdog_init(wdog_cfg_e cfg);   //��ʼ�����Ź�������ι��ʱ��
extern void wdog_feed(void);           //ι��


extern void wdog_disable(void);        //���ÿ��Ź�
extern void wdog_enable(void);         //���ÿ��Ź�

/********************************************************************/
#endif /* __MKL26_WDOG_H__ */
