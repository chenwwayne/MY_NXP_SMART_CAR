#include "common.h"
#include "include.h"
#include "isr.h"

extern uint8_t ADdata[DATALINE][DATACOUNT];
extern uint8_t  DMA_Over_Flag; 
uint8 row=0;
uint16 Sample_point;

uint16_t LinCout =0;
uint8_t  LinADCout=0;



void dlay(void)
{
  volatile uint16_t  dlaycount ;
   
  for(dlaycount = 0 ; dlaycount < 48; dlaycount ++){}
}

//�����ж�
void PORTA_IRQHandler()
{
  uint8_t  n = 0;    //���ź� 
  n=1;
  if(PORTA_ISFR & (1 << n))         //���ж�PTA1�����ж�
   {
       PORTA_ISFR  |= (1 << n);        //д1���жϱ�־λ   [LinADCout]
//      uart_putchar(UART4,LinCout>>8);      //��������
//      uart_putchar(UART4,LinCout);         //��������
         LinCout = 0 ;
         LinADCout=0;
         port_init (PTA6,  IRQ_FALLING| PF | ALT1 | PULLUP );       //�����ж�  ������������Ҫ��Ҫ�����������жϣ���Ҫֱ�ӿ�PORTA���жϣ�        
//			PORTA_ISFR  |=(1<<n); 
    }
  
       
      n=6;
      if(PORTA_ISFR & (1 << n))         //��PTA6�����ж�
        PORTA_ISFR  |= (1 << n);        //д1���жϱ�־λ   ADdata[LinCout]
        LinCout ++ ; 
		  DMA_IRQ_CLEAN(DMA_CH2);                             //���ͨ�������жϱ�־λ    (���������ٴν����ж�)
        if((LinCout%(400/DATALINE)==0)&&(LinADCout<DATALINE))
        {
			 dlay() ;
			 DMA_IRQ_CLEAN(DMA_CH2) ;
			 DMA_SAR(DMA_CH2) = (uint32_t)(&PTE_B2_IN) ; 
          DMA_DAR(DMA_CH2) = (uint32)(&ADdata[ LinADCout][0]);
			 DMA_DSR_BCR(DMA_CH2) =( 0 | DMA_DSR_BCR_BCR(DATACOUNT))  ;
			 DMA_DCR(DMA_CH2) |= DMA_DCR_ERQ_MASK ; 	; 
          DMA_EN(DMA_CH2);                                    //ʹ��ͨ��CHn Ӳ������      (�������ܼ�������DMA����) 
          DMA_IRQ_EN(DMA_CH2) ;                             //����DMAͨ������
			 LinADCout ++ ;
		  }
      
        if(LinADCout==DATALINE)
        	DMA_Over_Flag = 1 ;
}

void DMA2_IRQHandler()
{  
      DMA_IRQ_CLEAN(DMA_CH2) ;
      DMA_IRQ_DIS(DMA_CH2);
      DMA_DIS(DMA_CH2);
      Sample_point++;
      row++;	  			        	    		 	  	    		 
      dma_repeat(DMA_CH2,(void *)&PTE_B2_IN, ADdata[row],DATACOUNT);       
}


uint8 OLED_Dis_flag=1;
extern uint8 Stop_flag;
//extern uint32 Motor_Duty;
uint32 Pluse_feedback;
int debugTimer=400;
extern struct pid sPID;
extern uint8 obstacleLeft_flag;
extern uint8 obstacleRight_flag;
extern uint8 obstacle_flag;
extern uint8 reverse_flag;
extern uint8 first_clear;
extern int moveDelay;
extern uint8 debugtimer_flag;
int cnt=0;
/*sPID.ActualSpeed=(int)(Pluse_feedback*����һȦ����*{1/��ʱʱ�䣨ms��}/����һȦ����);  //��λΪcm/s*/
/*                                      18cm                20ms            1346                  */
/*��IAR���ߵ��Թ����У������Ȼת��ʱ�����ܹص����Դ���أ����򵱵��ֹͣ��Stop_flag��ȻΪ0����ʱ����¼�൱��û�г�ʼ�����pwm��Ϊ0����������ռ�ձ�ת��������*/
void PIT_IRQHandler(void)
{
  static int time_count=0;
  static int moveDelayCount=0;
  static int stop_debug_count=0;
//  static int cnt=0;
  if(PIT_TFLG(PIT0)==1)
  {
	  //���������巴��
	  Pluse_feedback=tpm_pulse_get(TPM1);
	  tpm_pulse_clean(TPM1);
	  sPID.ActualSpeed=Pluse_feedback*18*100/1346;//���廻���ʵ���ٶ�
	  
	  
	  cnt++;
	  //�����һ�β����
	  if(first_clear==1)
     {
     	  time_count++;
        if(time_count==100)//��һ�δ�������1000ms�����ٴδ���
        {
        		time_count = 0;
        		first_clear= 0;
      	}
     }
	  
	  //ͣ���߷�תͣ��
	  if(reverse_flag==1)
     { 
        if(sPID.ActualSpeed<10)
        {
            tpm_pwm_duty(TPM0, TPM_CH1, 0);
            tpm_pwm_duty(TPM0, TPM_CH2, 0);
            reverse_flag=0;
            Stop_flag=1;
         }
         else
         {
            tpm_pwm_duty(TPM0, TPM_CH1, 0);
            tpm_pwm_duty(TPM0, TPM_CH2,2500);
          }
      }
	  
	  //��Ȼͣ��
	  else if(Stop_flag)  
	  {
		  OLED_Dis_flag=1;
		  tpm_pwm_duty(TPM0, TPM_CH1, 0);
		  tpm_pwm_duty(TPM0, TPM_CH2, 0);
		  cnt=0;
	  }
	  
	  //�ٶȿ���
	  else 
	  {
//		 cnt++;
		 OLED_Dis_flag=0;
		 /*���̶����ռ�ձ���*/
//		 tpm_pwm_duty(TPM0, TPM_CH1, Motor_Duty);
//		 tpm_pwm_duty(TPM0, TPM_CH2, 0);
		  SpeedCtrl();
		  if(cnt>=50)//ֻ�ڸշ���ʱ��������
		  {
		  		Motor_Stall();
				cnt=50;//��ֹ���
		  }
		  /*����ʾ�����������������ﵥƬ���Ῠס���ص������жϺ���ԣ���������������������Ͳ��Σ��ùص������жϣ�*/
        /*send_buff[0]=(uint32)sPID.ActualSpeed;*/
		  /*send_buff[1]=(uint32)Pluse_feedback;*/
		  /*vcan_sendware((uint8*)send_buff,8);*/
	  }
	  
	  	//��ʱ����
	  if(moveDelay==1)
	  {
	  		moveDelayCount++;
			if(moveDelayCount==200) //2000ms
			{
				moveDelayCount=0;//��ʱʱ���������
				moveDelay=0;//��־λ����
				Stop_flag=0;
			}
	  }
	  if(Stop_flag==0)
	  {
		 stop_debug_count++;
		 if(stop_debug_count==debugTimer)
		 {
			  debugtimer_flag=1;
			  stop_debug_count=0;
		 }
	  }
	  //���жϱ�־λ
	  PIT_TFLG(PIT0)|=PIT_TFLG_TIF_MASK;
	}
  
   if(PIT_TFLG(PIT1)==1)
	{
	  obstacleLeft_flag=obstacleRight_flag=0;
	  obstacle_flag=0;
	  gpio_set(PTB2,1);
	  PIT_TFLG(PIT1)|=PIT_TFLG_TIF_MASK;
	}
}
