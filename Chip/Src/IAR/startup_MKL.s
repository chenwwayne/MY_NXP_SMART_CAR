 /*!
  *     COPYRIGHT NOTICE
  *     Copyright (c) 2013,ɽ��Ƽ�
  *     All rights reserved.
  *     �������ۣ�ɽ����̳ http://www.vcan123.com
  *
  *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
  *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
  *
  * @file       startup_MKL.s
  * @brief      ϵͳ������λ����
  * @author     ɽ��Ƽ�
  * @version    v5.0
  * @date       2013-11-19
  */



    SECTION .noinit : CODE          ; //ָ������Σ�.noinit
    EXPORT  Reset_Handler           ; //���� Reset_Handler ����
Reset_Handler
    CPSIE   i                       ; //ʹ��ȫ���ж�
    import start                    ; //��������
    BL      start                   ; //���� C���Ժ��� start
__done
    B       __done


        END
