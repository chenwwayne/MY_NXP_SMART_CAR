/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MKL_gpio.c
 * @brief      gpio��������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-06-26
 */

/*
 * ����ͷ�ļ�
 */
#include "common.h"
#include "MKL_port.h"
#include "MKL_gpio.h"

/*
 * ��������
 */
#if 1   //���� GPIO ���� ���� ����һ�� �� ����IO ���ڶ����� ��ͨIO
GPIO_MemMapPtr GPIOX[PTX_MAX] = {(GPIO_MemMapPtr)FPTA_BASE_PTR, (GPIO_MemMapPtr)FPTB_BASE_PTR, (GPIO_MemMapPtr)FPTC_BASE_PTR, (GPIO_MemMapPtr)FPTD_BASE_PTR, (GPIO_MemMapPtr)FPTE_BASE_PTR}; //�������ָ�����鱣�� GPIOX �ĵ�ַ
#else
GPIO_MemMapPtr GPIOX[PTX_MAX] = {PTA_BASE_PTR,PTB_BASE_PTR,PTC_BASE_PTR, PTD_BASE_PTR, PTE_BASE_PTR}; //�������ָ�����鱣�� GPIOX �ĵ�ַ

#endif


/*!
 *  @brief      ��ʼ��gpio
 *  @param      PTxn    �˿�
 *  @param      cfg     ���ŷ���,0=����,1=���
 *  @param      data    �����ʼ״̬,0=�͵�ƽ,1=�ߵ�ƽ ����������Ч��
 *  @since      v5.0
 *  Sample usage:       gpio_init (PTA8, GPI,0);    //��ʼ�� PTA8 �ܽ�Ϊ����
 */
void gpio_init (PTXn_e ptxn, GPIO_CFG cfg, uint8 data)
{


    //�˿ڷ���������뻹�����
    if(  cfg == GPI )
    {
        //���ö˿ڷ���Ϊ����
        GPIO_PDDR_REG(GPIOX_BASE(ptxn)) &= ~(1 << PTn(ptxn));       // GPIO PDDR �ܽź� ��0������Ӧ�ܽ�����Ϊ�˿ڷ�������
    }
    else
    {
        //���ö˿ڷ���Ϊ���
        GPIO_PDDR_REG(GPIOX_BASE(ptxn)) |= (1 << PTn(ptxn));        // GPIO PDDR �ܽź� ��1������Ӧ�ܽ�����Ϊ�˿ڷ������

        //�˿��������
        if(data == 0)
        {
            GPIO_PDOR_REG(GPIOX_BASE(ptxn)) &= ~(1 << PTn(ptxn));   // GPIO PDOR �ܽź� ��0������Ӧ�ܽ�����Ϊ�˿�����͵�ƽ
        }
        else
        {
            GPIO_PDOR_REG(GPIOX_BASE(ptxn))  |= (1 << PTn(ptxn));   // GPIO PDOR �ܽź� ��1������Ӧ�ܽ�����Ϊ�˿�����ߵ�ƽ
        }
    }

    //���ùܽ�ΪGPIO����
    port_init( ptxn, ALT1);
}

/*!
 *  @brief      �����������ݷ���
 *  @param      PTxn    �˿�
 *  @param      cfg     ���ŷ���,0=����,1=���
 *  @since      v5.0
 *  Sample usage:       gpio_ddr (PTA8, GPI);    //���� PTA8 �ܽ�Ϊ����
 */
void    gpio_ddr   (PTXn_e ptxn, GPIO_CFG cfg)
{
    //�˿ڷ���������뻹�����
    if(  cfg == GPI )
    {
        //���ö˿ڷ���Ϊ����
        GPIO_PDDR_REG(GPIOX_BASE(ptxn)) &= ~(1 << PTn(ptxn));           // GPIO PDDR �ܽź� ��0������Ӧ�ܽ�����Ϊ�˿ڷ�������
    }
    else
    {
        //���ö˿ڷ���Ϊ���
        GPIO_PDDR_REG(GPIOX_BASE(ptxn)) |= (1 << PTn(ptxn));            // GPIO PDDR �ܽź� ��1������Ӧ�ܽ�����Ϊ�˿ڷ������
    }
}

/*!
 *  @brief      ��������״̬
 *  @param      PTxn    �˿�
 *  @param      data    �����ʼ״̬,0=�͵�ƽ,1=�ߵ�ƽ ����������Ч��
 *  @since      v5.0
 *  @warning    ��ر�֤���ݷ���Ϊ�����DEBUGģʽ�£��ж��Խ��м�⣩
 *  Sample usage:       gpio_set (PTA8, 1);    // PTA8 �ܽ� ��� 1
 */
void gpio_set (PTXn_e ptxn, uint8 data)
{
    ASSERT( BIT_GET( GPIO_PDDR_REG(GPIOX_BASE(ptxn)) , PTn(ptxn)) == GPO ); // ���ԣ���� ��������Ƿ�Ϊ���
                                                                            // ��ȡ GPIO PDDR �ܽź� ���Ƚ��Ƿ�Ϊ���

    //�˿��������
    if(data == 0)
    {
        GPIO_PDOR_REG(GPIOX_BASE(ptxn)) &= ~(1 << PTn(ptxn));   // GPIO PDOR �ܽź� ��0������Ӧ�ܽ�����Ϊ�˿�����͵�ƽ
    }
    else
    {
        GPIO_PDOR_REG(GPIOX_BASE(ptxn))  |= (1 << PTn(ptxn));   // GPIO PDOR �ܽź� ��1������Ӧ�ܽ�����Ϊ�˿�����ߵ�ƽ
    }
}


/*!
 *  @brief      ��ת����״̬
 *  @param      PTxn    �˿�
 *  @since      v5.0
 *  @warning    ��ر�֤���ݷ���Ϊ�����DEBUGģʽ�£��ж��Խ��м�⣩
 *  Sample usage:       gpio_turn (PTA8);    // PTA8 �ܽ� ��� ��ת
 */
void gpio_turn (PTXn_e ptxn)
{
    ASSERT( BIT_GET( GPIO_PDDR_REG(GPIOX_BASE(ptxn)) , PTn(ptxn)) == GPO ); // ���ԣ���� ��������Ƿ�Ϊ���
                                                                            // ��ȡ GPIO PDDR �ܽź� ���Ƚ��Ƿ�Ϊ���

    GPIO_PTOR_REG( GPIOX_BASE(ptxn))  =  1 << (PTn(ptxn ));                 // GPIO PTOR ptxn ��1��������0 ������Ӧ�ܽ�����Ϊ�˿������ת������λ����
                                                                            // �˴������� BIT_SET ���������1 ����Ϊ���뱣֤����λ ���䣬����λֱ����0����
}

/*!
 *  @brief      ��ȡ��������״̬
 *  @param      PTxn    �˿�
 *  @return     �ܽŵ�״̬��1Ϊ�ߵ�ƽ��0Ϊ�͵�ƽ
 *  @since      v5.0
 *  @warning    ��ر�֤���ݷ���Ϊ���루DEBUGģʽ�£��ж��Խ��м�⣩
 *  Sample usage:       uint8 pta8_data = gpio_get (PTA8);    // ��ȡ PTA8 �ܽ� �����ƽ
 */
uint8 gpio_get(PTXn_e ptxn)
{
    ASSERT( BIT_GET( GPIO_PDDR_REG(GPIOX_BASE(ptxn)) , PTn(ptxn)) == GPI ); // ���ԣ���� ��������Ƿ�Ϊ����
                                                                            // ��ȡ GPIO PDDR �ܽź� ���Ƚ��Ƿ�Ϊ����

    return ((GPIO_PDIR_REG(GPIOX_BASE(ptxn)) >> PTn(ptxn )) & 0x01);        // ��ȡ GPIO PDIR ptxn ״̬������ȡ�ܽ������ƽ
}



