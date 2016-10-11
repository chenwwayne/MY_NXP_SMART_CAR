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

//场行中断
void PORTA_IRQHandler()
{
  uint8_t  n = 0;    //引脚号 
  n=1;
  if(PORTA_ISFR & (1 << n))         //场中断PTA1触发中断
   {
       PORTA_ISFR  |= (1 << n);        //写1清中断标志位   [LinADCout]
//      uart_putchar(UART4,LinCout>>8);      //采样行数
//      uart_putchar(UART4,LinCout);         //采样行数
         LinCout = 0 ;
         LinADCout=0;
         port_init (PTA6,  IRQ_FALLING| PF | ALT1 | PULLUP );       //开行中断  （！！！！重要，要用引脚来开中断，不要直接开PORTA口中断）        
//			PORTA_ISFR  |=(1<<n); 
    }
  
       
      n=6;
      if(PORTA_ISFR & (1 << n))         //行PTA6触发中断
        PORTA_ISFR  |= (1 << n);        //写1清中断标志位   ADdata[LinCout]
        LinCout ++ ; 
		  DMA_IRQ_CLEAN(DMA_CH2);                             //清除通道传输中断标志位    (这样才能再次进入中断)
        if((LinCout%(400/DATALINE)==0)&&(LinADCout<DATALINE))
        {
			 dlay() ;
			 DMA_IRQ_CLEAN(DMA_CH2) ;
			 DMA_SAR(DMA_CH2) = (uint32_t)(&PTE_B2_IN) ; 
          DMA_DAR(DMA_CH2) = (uint32)(&ADdata[ LinADCout][0]);
			 DMA_DSR_BCR(DMA_CH2) =( 0 | DMA_DSR_BCR_BCR(DATACOUNT))  ;
			 DMA_DCR(DMA_CH2) |= DMA_DCR_ERQ_MASK ; 	; 
          DMA_EN(DMA_CH2);                                    //使能通道CHn 硬件请求      (这样才能继续触发DMA传输) 
          DMA_IRQ_EN(DMA_CH2) ;                             //允许DMA通道传输
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
/*sPID.ActualSpeed=(int)(Pluse_feedback*轮子一圈距离*{1/定时时间（ms）}/轮子一圈脉冲);  //单位为cm/s*/
/*                                      18cm                20ms            1346                  */
/*在IAR在线调试过程中，电机自然转动时，不能关电机电源开关，否则当电机停止后，Stop_flag仍然为0，此时再烧录相当于没有初始化电机pwm都为0，则电机会满占空比转动！！！*/
void PIT_IRQHandler(void)
{
  static int time_count=0;
  static int moveDelayCount=0;
  static int stop_debug_count=0;
//  static int cnt=0;
  if(PIT_TFLG(PIT0)==1)
  {
	  //编码器脉冲反馈
	  Pluse_feedback=tpm_pulse_get(TPM1);
	  tpm_pulse_clean(TPM1);
	  sPID.ActualSpeed=Pluse_feedback*18*100/1346;//脉冲换算成实际速度
	  
	  
	  cnt++;
	  //红外第一次不检测
	  if(first_clear==1)
     {
     	  time_count++;
        if(time_count==100)//第一次触发后间隔1000ms才能再次触发
        {
        		time_count = 0;
        		first_clear= 0;
      	}
     }
	  
	  //停车线反转停车
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
	  
	  //自然停车
	  else if(Stop_flag)  
	  {
		  OLED_Dis_flag=1;
		  tpm_pwm_duty(TPM0, TPM_CH1, 0);
		  tpm_pwm_duty(TPM0, TPM_CH2, 0);
		  cnt=0;
	  }
	  
	  //速度控制
	  else 
	  {
//		 cnt++;
		 OLED_Dis_flag=0;
		 /*给固定电机占空比用*/
//		 tpm_pwm_duty(TPM0, TPM_CH1, Motor_Duty);
//		 tpm_pwm_duty(TPM0, TPM_CH2, 0);
		  SpeedCtrl();
		  if(cnt>=50)//只在刚发车时候起作用
		  {
		  		Motor_Stall();
				cnt=50;//防止溢出
		  }
		  /*虚拟示波器发送数据在这里单片机会卡住，关掉场行中断后可以，所以如果放在这里来发送波形，得关掉场行中断！*/
        /*send_buff[0]=(uint32)sPID.ActualSpeed;*/
		  /*send_buff[1]=(uint32)Pluse_feedback;*/
		  /*vcan_sendware((uint8*)send_buff,8);*/
	  }
	  
	  	//延时启动
	  if(moveDelay==1)
	  {
	  		moveDelayCount++;
			if(moveDelayCount==200) //2000ms
			{
				moveDelayCount=0;//延时时间记数清零
				moveDelay=0;//标志位清零
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
	  //清中断标志位
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
