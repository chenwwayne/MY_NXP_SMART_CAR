//========================================================
//          集美大学简答题全队摄像头程序
//          队员：林荣关，邱伯怀，陈炜炜
//                                      2016.7.16
//========================================================
#include "common.h"
#include "include.h"
/*采集图像用到的参数*/
uint8_t ADdata[DATALINE][DATACOUNT] ={0} ; //原始图像数组存储
uint8_t Data[DATALINE][DATACOUNT]={0} ;//二值化图像数组存储
uint8_t threshold=120;
//uint8_t  checkflag = 0 ; //检查是否完成I2C写入配置标志位
/*寻线用到的参数*/
int crossroad_flag=0;
/*舵机用到的参数*/
uint32 Servo_Duty;
uint32 Servo_Duty_median=4530;
extern float Servo_Err;
/*电机用到的参数*/
//uint32 Motor_Duty=2500;
/*拨码开关状态标志位*/
uint8 Switch1_State=0;
uint8 Switch2_State=0;
uint8 Switch3_State=0;
uint8 Switch4_State=0;
uint8 Switch5_State=0;
/*调参数用*/
extern uint8 OLED_Dis_flag;
uint8 Stop_flag=1;
uint8 OLED_Mode=0;
//extern uint8 send_buff[2];
extern struct pid sPID;//电机PID结构体
/*程序测时用*/
//uint32 time_vlaue;
/*OLED显示用*/
uint8 tempimg[40][128];
extern int Black_Center;
/*红外检测停车线用*/
//uint8 infrared_stop_R;
//uint8 infrared_stop_L;
uint8 reverse_flag=0;
int first_clear=0;
extern uint8 obstacle_flag;
/*定时停车，调试用*/
uint8 debugtimer_flag=0;
/*全黑停车*/
//extern int outStop_flag;

void All_Init(void);
void Bluetooth_Ctrl(char *buff);
void InfraredStop(void);
void main()
{	
  	char bluetooth_ch;//蓝牙接收值
   All_Init();
   while(1)
   {     
	  	Swithch_Scan();
		Bluetooth_Ctrl(&bluetooth_ch);//蓝牙控制函数
		Get_IMG();
		if(OLED_Dis_flag)
	   {
			Matrix_KeyScan();
			if(OLED_Mode==1)//显示参数		
			{
				OLED_Display();
			}
			if(OLED_Mode==0) OLED_IMG(tempimg,Black_Center);//显示赛道	
		}
		if(Switch4_State==1)
		{
		  InfraredStop();//红外停车
		}
		if(Switch3_State==1)//定时停车，默认停车时间在中断里面修改
		{
			if(debugtimer_flag==1)
			{
				Stop_flag=1;
				debugtimer_flag=0;
			}
		}
//		if(outStop_flag==1)   用于全黑行停车
//		{
//			Stop_flag=1;
//		   outStop_flag=0;
//		}
	}
}

/*红外停车实现函数*/
void InfraredStop(void)
{
	uint8 infrared_stop_R_DW;
   uint8 infrared_stop_L_DW;
   uint8 infrared_stop_R_UP;
   uint8 infrared_stop_L_UP;
	static int second_flag=0;//第二次触发停车 
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
           first_clear = 1;//在中断里计数1s后清零
           if(second_flag==2)//第二次触发停车
           {
              second_flag = 0;//标志位复位
              reverse_flag= 1;//反转标志位
           } 
        }  
      }
   }
}

void All_Init(void)
{ 
  /*指示灯初始化*/      
  led_init(LED0);
  led_init(LED1);
  led_init(LED2);
  led_init(LED3);
  
  /*拨码开关初始化*/
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
  
   /*蜂鸣器初始化*/
   gpio_init(PTB2, GPO, 1);
   port_init_NoALT(PTB2, PULLUP);
  
  /*OLED初始化*/
  OLED_Init();  
  
  /*摄像头管脚初始化*/
  LandzoCamera_init();
  
  /*计数初始化*/ 
//  pit_time_start(PIT1);
//  PIT_TCTRL(PIT1) &= ~ PIT_TCTRL_TEN_MASK;//关闭计数
  
  /*设置中断优先级*/
  set_irq_priority(PORTA_IRQn,1);
  set_irq_priority(PIT_IRQn,2);
  
  /*定时中断初始化*/
   pit_init_ms(PIT0,10);
//	pit_init_ms(PIT1,20);//如果初始化PIT1,但中断里面却没写PIT1的执行函数，单片机会卡住
	set_vector_handler(PIT_VECTORn,PIT_IRQHandler);
   enable_irq(PIT_IRQn); 
  
  /*串口初始化*/
   uart_init (UART0,115200);//如果蓝牙不能用，在初始化蓝牙之前延时几ms可恢复正常
  
   /* IIC配置摄像头*/
//       while(checkflag != 1 )  
//       {
//            checkflag = LandzoIICEEROM_INIT() ;
//            BFdelay_1us(100);
//            // 延时100us 
//            led (LED0,LED_ON);
//       }

  /*图像采集初始化*/
  set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);//设置场中断服务函数
  enable_irq(PORTA_IRQn); 
  set_vector_handler(DMA2_VECTORn,DMA2_IRQHandler);
  enable_irq(DMA2_IRQn);
      
   /*图像数据初始化*/
   IMG_Init();
   
   /*电机PWM初始化*/
   tpm_pwm_init(TPM0,TPM_CH1,10*1000,0);
   tpm_pwm_init(TPM0,TPM_CH2,10*1000,0);
   
   /*舵机PWM初始化*/
   tpm_pwm_init(TPM2,TPM_CH1,300,Servo_Duty_median);
	
	/*编码器采集初始化*/
   tpm_pulse_init(TPM1,TPM_CLKIN0,TPM_PS_1);
   port_init_NoALT (PTB16, PF | PULLUP );
   
	/*速度PID参数初始化*/
	PID_Init();
	
	/*红外端口初始化*/
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

/*红外控制函数*/
void Bluetooth_Ctrl(char *buff)
{	
	uart_querychar (UART0, buff);
	if(*buff==01){*buff=0;Stop_flag=1;}//停车
//	if(*buff==02){*buff=0;Motor_Duty=2300;}
// if(*buff==03){*buff=0;Motor_Duty=2500;}
// if(*buff==04){*buff=0;Motor_Duty=2700;}
//	if(*buff==05){*buff=0;Motor_Duty=2900;}
//	if(*buff==06){*buff=0;Motor_Duty=3100;}
//	if(*buff==07){*buff=0;Motor_Duty=3300;}
}

  



