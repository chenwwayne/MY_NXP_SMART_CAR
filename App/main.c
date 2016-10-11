//========================================================
//          ������ѧ�����ȫ������ͷ����
//          ��Ա�����ٹأ��񲮻�������
//                                      2016.7.16
//========================================================
#include "common.h"
#include "include.h"
/*�ɼ�ͼ���õ��Ĳ���*/
uint8_t ADdata[DATALINE][DATACOUNT] ={0} ; //ԭʼͼ������洢
uint8_t Data[DATALINE][DATACOUNT]={0} ;//��ֵ��ͼ������洢
uint8_t threshold=120;
//uint8_t  checkflag = 0 ; //����Ƿ����I2Cд�����ñ�־λ
/*Ѱ���õ��Ĳ���*/
int crossroad_flag=0;
/*����õ��Ĳ���*/
uint32 Servo_Duty;
uint32 Servo_Duty_median=4530;
extern float Servo_Err;
/*����õ��Ĳ���*/
//uint32 Motor_Duty=2500;
/*���뿪��״̬��־λ*/
uint8 Switch1_State=0;
uint8 Switch2_State=0;
uint8 Switch3_State=0;
uint8 Switch4_State=0;
uint8 Switch5_State=0;
/*��������*/
extern uint8 OLED_Dis_flag;
uint8 Stop_flag=1;
uint8 OLED_Mode=0;
//extern uint8 send_buff[2];
extern struct pid sPID;//���PID�ṹ��
/*�����ʱ��*/
//uint32 time_vlaue;
/*OLED��ʾ��*/
uint8 tempimg[40][128];
extern int Black_Center;
/*������ͣ������*/
//uint8 infrared_stop_R;
//uint8 infrared_stop_L;
uint8 reverse_flag=0;
int first_clear=0;
extern uint8 obstacle_flag;
/*��ʱͣ����������*/
uint8 debugtimer_flag=0;
/*ȫ��ͣ��*/
//extern int outStop_flag;

void All_Init(void);
void Bluetooth_Ctrl(char *buff);
void InfraredStop(void);
void main()
{	
  	char bluetooth_ch;//��������ֵ
   All_Init();
   while(1)
   {     
	  	Swithch_Scan();
		Bluetooth_Ctrl(&bluetooth_ch);//�������ƺ���
		Get_IMG();
		if(OLED_Dis_flag)
	   {
			Matrix_KeyScan();
			if(OLED_Mode==1)//��ʾ����		
			{
				OLED_Display();
			}
			if(OLED_Mode==0) OLED_IMG(tempimg,Black_Center);//��ʾ����	
		}
		if(Switch4_State==1)
		{
		  InfraredStop();//����ͣ��
		}
		if(Switch3_State==1)//��ʱͣ����Ĭ��ͣ��ʱ�����ж������޸�
		{
			if(debugtimer_flag==1)
			{
				Stop_flag=1;
				debugtimer_flag=0;
			}
		}
//		if(outStop_flag==1)   ����ȫ����ͣ��
//		{
//			Stop_flag=1;
//		   outStop_flag=0;
//		}
	}
}

/*����ͣ��ʵ�ֺ���*/
void InfraredStop(void)
{
	uint8 infrared_stop_R_DW;
   uint8 infrared_stop_L_DW;
   uint8 infrared_stop_R_UP;
   uint8 infrared_stop_L_UP;
	static int second_flag=0;//�ڶ��δ���ͣ�� 
	if(fabs(Servo_Err)<15)
	{      
      if(gpio_get(PTB20)==1)	infrared_stop_R_UP=1;
		else infrared_stop_R_UP=0;
      if(gpio_get(PTB21)==1)	infrared_stop_L_UP=1;
		else infrared_stop_L_UP=0;
      if(gpio_get(PTC18)==1)	infrared_stop_L_DW=1;
		else infrared_stop_L_DW=0;
      if(gpio_get(PTA7)==1)	infrared_stop_R_DW=1;
		else infrared_stop_R_DW=0;
		if((infrared_stop_L_UP==1||infrared_stop_L_DW==1)&&(infrared_stop_R_UP==1||infrared_stop_R_DW==1)&&Stop_flag==0 &&obstacle_flag==0)
      {
        if(first_clear== 0)
        {
           second_flag++;
           first_clear = 1;//���ж������1s������
           if(second_flag==2)//�ڶ��δ���ͣ��
           {
              second_flag = 0;//��־λ��λ
              reverse_flag= 1;//��ת��־λ
           } 
        }  
      }
   }
}

void All_Init(void)
{ 
  /*ָʾ�Ƴ�ʼ��*/      
  led_init(LED0);
  led_init(LED1);
  led_init(LED2);
  led_init(LED3);
  
  /*���뿪�س�ʼ��*/
  gpio_init(PTD1, GPI, 0);
  port_init_NoALT(PTD1, PULLUP);
  gpio_init(PTC17, GPI, 0);
  port_init_NoALT(PTC17, PULLUP);
  gpio_init(PTC13, GPI, 0);
  port_init_NoALT(PTC13, PULLUP);
  gpio_init(PTD2, GPI, 0);
  port_init_NoALT(PTD2, PULLUP);
  gpio_init(PTD3, GPI, 0);
  port_init_NoALT(PTD3, PULLUP);
  
   /*��������ʼ��*/
   gpio_init(PTB2, GPO, 1);
   port_init_NoALT(PTB2, PULLUP);
  
  /*OLED��ʼ��*/
  OLED_Init();  
  
  /*����ͷ�ܽų�ʼ��*/
  LandzoCamera_init();
  
  /*������ʼ��*/ 
//  pit_time_start(PIT1);
//  PIT_TCTRL(PIT1) &= ~ PIT_TCTRL_TEN_MASK;//�رռ���
  
  /*�����ж����ȼ�*/
  set_irq_priority(PORTA_IRQn,1);
  set_irq_priority(PIT_IRQn,2);
  
  /*��ʱ�жϳ�ʼ��*/
   pit_init_ms(PIT0,10);
//	pit_init_ms(PIT1,20);//�����ʼ��PIT1,���ж�����ȴûдPIT1��ִ�к�������Ƭ���Ῠס
	set_vector_handler(PIT_VECTORn,PIT_IRQHandler);
   enable_irq(PIT_IRQn); 
  
  /*���ڳ�ʼ��*/
   uart_init (UART0,115200);//������������ã��ڳ�ʼ������֮ǰ��ʱ��ms�ɻָ�����
  
   /* IIC��������ͷ*/
//       while(checkflag != 1 )  
//       {
//            checkflag = LandzoIICEEROM_INIT() ;
//            BFdelay_1us(100);
//            // ��ʱ100us 
//            led (LED0,LED_ON);
//       }

  /*ͼ��ɼ���ʼ��*/
  set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);//���ó��жϷ�����
  enable_irq(PORTA_IRQn); 
  set_vector_handler(DMA2_VECTORn,DMA2_IRQHandler);
  enable_irq(DMA2_IRQn);
      
   /*ͼ�����ݳ�ʼ��*/
   IMG_Init();
   
   /*���PWM��ʼ��*/
   tpm_pwm_init(TPM0,TPM_CH1,10*1000,0);
   tpm_pwm_init(TPM0,TPM_CH2,10*1000,0);
   
   /*���PWM��ʼ��*/
   tpm_pwm_init(TPM2,TPM_CH1,300,Servo_Duty_median);
	
	/*�������ɼ���ʼ��*/
   tpm_pulse_init(TPM1,TPM_CLKIN0,TPM_PS_1);
   port_init_NoALT (PTB16, PF | PULLUP );
   
	/*�ٶ�PID������ʼ��*/
	PID_Init();
	
	/*����˿ڳ�ʼ��*/
	gpio_init(PTB20, GPI, 0);
   port_init_NoALT(PTB20, PULLUP);
	gpio_init(PTB21, GPI, 0);
   port_init_NoALT(PTB21, PULLUP);
   gpio_init(PTA7, GPI, 0);
   port_init_NoALT(PTA7, PULLUP);
	gpio_init(PTC18, GPI, 0);
   port_init_NoALT(PTC18, PULLUP);
//	gpio_init(PTA7, GPI, 0);
//   port_init_NoALT(PTA7, PULLUP);
//	gpio_init(PTC18, GPI, 0);
//   port_init_NoALT(PTC18, PULLUP);
  
}

/*������ƺ���*/
void Bluetooth_Ctrl(char *buff)
{	
	uart_querychar (UART0, buff);
	if(*buff==01){*buff=0;Stop_flag=1;}//ͣ��
//	if(*buff==02){*buff=0;Motor_Duty=2300;}
// if(*buff==03){*buff=0;Motor_Duty=2500;}
// if(*buff==04){*buff=0;Motor_Duty=2700;}
//	if(*buff==05){*buff=0;Motor_Duty=2900;}
//	if(*buff==06){*buff=0;Motor_Duty=3100;}
//	if(*buff==07){*buff=0;Motor_Duty=3300;}
}

  



