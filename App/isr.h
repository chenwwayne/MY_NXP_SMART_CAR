
#ifndef __ISR_H
#define __ISR_H 1

#include  "include.h"


extern void PORTC_PORTD_IRQHandler();           //�г��жϷ�����

extern void PORTA_IRQHandler();//���жϡ����жϺ�������
extern void PIT_IRQHandler();

extern void DMA2_IRQHandler();
extern void vcan_sendware(uint8 *wareaddr, uint32 waresize);

#undef VECTOR_016
#define VECTOR_016  DMA3_IRQHandler

#endif  //__ISR_H

/* End of "isr.h" */