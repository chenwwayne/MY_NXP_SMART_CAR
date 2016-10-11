#include "common.h"
#include "include.h"
#include "Key&Oled.h"

/*拨码开关状态标志位*/
extern uint8 Switch1_State;
extern uint8 Switch2_State;
extern uint8 Switch3_State;
extern uint8 Switch4_State;
extern uint8 Switch5_State;
/*调试用*/
extern float loca_Kp_general;
extern float loca_Kd_general;
int8 mode=0;
extern uint32 Servo_Duty_median;
extern uint32 Servo_Duty;
extern uint8_t threshold;
extern uint8 Stop_flag;
extern uint8 OLED_Mode;
extern uint8 Start_Center ;//开始中心处理的行
extern uint8 End_Center;//结束中心处理的行
//extern uint32 Motor_Duty;
extern float Servo_Err;
extern struct pid sPID;
extern float set_speed;
extern double set_sp_Kp;
extern float set_sp_Ki;
extern double set_sp_Kd;
extern float slope;
extern uint16 delayTime;
uint8 z=1;
int moveDelay=0;
extern int debugTimer;

void Swithch_Scan()
{  
  uint8 Switch1=gpio_get(PTD2);
  uint8 Switch2=gpio_get(PTD3);
  uint8 Switch3=gpio_get(PTD1);
  uint8 Switch4=gpio_get(PTC17);
  uint8 Switch5=gpio_get(PTC13);
  
  if(Switch1==0)  
	{
		if(z==1) OLED_CLS();
		z=2;
		OLED_Mode=1;
	 }       
	 else 
	 {
		if(z==2) OLED_CLS();
		z=1;
		OLED_Mode=0;
	 }
  if(Switch2==0)  {Switch2_State=1;}   else {Switch2_State=0;}
  if(Switch3==0)  {Switch3_State=1;}   else {Switch3_State=0;}
  if(Switch4==0)  {Switch4_State=1;}   else {Switch4_State=0;}
  if(Switch5==0)  {Switch5_State=1;}   else {Switch5_State=0;}
}

void OLED_Display(void)
{
	if(mode==0)
{
	   OLED_Print_Str(41,0,"MODE0");
		
	   OLED_Print_Str(0,2,"loca_Kp_g:");
      Dis_Float(2,43,loca_Kp_general,1);
		
		OLED_Print_Str(0,4,"loca_Kd_g:");
      Dis_Float(4,43,loca_Kd_general,1);
		
		OLED_Print_Str(0,6,"S_D_mid:");
      Dis_Float(6,43,Servo_Duty_median,0);
}
	 else if(mode==1)
	{
	  
		OLED_Print_Str(41,0,"MODE1");
		
		OLED_Print_Str(0,2,"threshold:");
      Dis_Float(2,44,threshold,0);
		
		OLED_Print_Str(0,4,"Start_C:");
      Dis_Float(4,45,Start_Center,0);
		
		OLED_Print_Str(0,6,"End_C:");
      Dis_Float(6,45,End_Center,0);
	}
	else if(mode==2)
	{
	  
		OLED_Print_Str(41,0,"MODE2");
		
	   OLED_Print_Str(0,2,"set_speed:");
      Dis_Float(2,44,set_speed,0);
		  
		OLED_Print_Str(0,4,"set_sp_Ki:");
      Dis_Float(4,44,set_sp_Ki,1);
		
		OLED_Print_Str(0,6,"set_sp_Kp:");
      Dis_Float(6,44,set_sp_Kp,1);
		
		
	}
	else if(mode==3)
	{
	  OLED_Print_Str(41,0,"MODE3");
	  
	  OLED_Print_Str(0,2,"set_sp_Kd:");
     Dis_Float(2,43,set_sp_Kd,1);
	  
	  OLED_Print_Str(0,4,"debugTimer:");
     Dis_Float(4,43,debugTimer,1);
	  
	}
	else if(mode==4)
	{
	  OLED_Print_Str(10,0,"MODE4_display");
	  
	  OLED_Print_Str(0,2,"Servo_Err:");
     Dis_Float(2,43,fabs(Servo_Err),2);
	  
	  OLED_Print_Str(0,4,"slope:");
     Dis_Float(4,40,fabs(slope),2);
		 
	  OLED_Print_Str(0,6,"delay_T:");
     Dis_Float(6,40,delayTime,1);
	}
}

uint8 Matrix_Key(void)
{ 
  uint8 code_h;
  uint8 code_l;
  /*PTC10 8 6（D5 D4 D3） 输出0   列*/
  
  gpio_init(PTC10,GPO,0);
//  port_init_NoALT(PTC10, PULLUP);
  gpio_init(PTC8, GPO,0);
//  port_init_NoALT(PTC8, PULLUP);
  gpio_init(PTC6, GPO,0);
//  port_init_NoALT(PTC6, PULLUP);
  //PTC4 3 1 (D2 D1 D0) 输入1   行
  gpio_init(PTC4, GPI,1);
  port_init_NoALT(PTC4, PULLUP);
  gpio_init(PTC3, GPI,1);
  port_init_NoALT(PTC3, PULLUP);
  gpio_init(PTC1, GPI,1);
  port_init_NoALT(PTC1, PULLUP);
  
  if(gpio_get(PTC4)==0||gpio_get(PTC3)==0||gpio_get(PTC1)==0)
  { 
  	systick_delay_us(5);
	if(gpio_get(PTC4)==0||gpio_get(PTC3)==0||gpio_get(PTC1)==0)
	{
        if(gpio_get(PTC1)==0&&gpio_get(PTC3)!=0&&gpio_get(PTC4)!=0)//D0=0
          code_h=01;
        else if(gpio_get(PTC1)!=0&&gpio_get(PTC3)==0&&gpio_get(PTC4)!=0)//D1=0
          code_h=02;
        else if(gpio_get(PTC1)!=0&&gpio_get(PTC3)!=0&&gpio_get(PTC4)==0)//D2=0
          code_h=03;
     
      /*PTC10 8 6（D5 D4 D3） 输出1   列*/
      gpio_init(PTC10,GPI,1);
      port_init_NoALT(PTC10, PULLUP);
      gpio_init(PTC8, GPI,1);
      port_init_NoALT(PTC8, PULLUP);
      gpio_init(PTC6, GPI,1);
      port_init_NoALT(PTC6, PULLUP);
      //PTC4 3 1 (D2 D1 D0) 输入0  行
      gpio_init(PTC4, GPO,0);
//      port_init_NoALT(PTC4, PULLUP);
      gpio_init(PTC3, GPO,0);
//      port_init_NoALT(PTC3, PULLUP);
      gpio_init(PTC1, GPO,0);
//      port_init_NoALT(PTC1, PULLUP);
       
     if(gpio_get(PTC6)==0&&gpio_get(PTC8)!=0&&gpio_get(PTC10)!=0)//D3=0
        code_l=10;
     else if(gpio_get(PTC6)!=0&&gpio_get(PTC8)==0&&gpio_get(PTC10)!=0)//D4=0
        code_l=20;
     else if(gpio_get(PTC6)!=0&&gpio_get(PTC8)!=0&&gpio_get(PTC10)==0)//D5=0
        code_l=30;
     
     while(gpio_get (PTC10)==0 || gpio_get (PTC8)==0 || gpio_get (PTC6)==0);//等待松开并输出
     
     
     }
     
  }
  return(code_h+code_l);
}


void Matrix_KeyScan(void)//改
{		
    switch(Matrix_Key())
    {		 
	 		 case 11: {
				if(mode==0)
				{
					Servo_Duty_median-=10;
				}
				else if(mode==1)
				{
					End_Center=End_Center-1;
				}
				else if(mode==2)
				{
					set_sp_Kp-=1;
				}
				else if(mode==4)
				{
					delayTime-=10;
				}
				break;}//k7
			 
          case 12: {
				if(mode==0)
				{
					Servo_Duty_median+=10;
				}
				else if(mode==1)
				{
					End_Center=End_Center+1;
				}
				else if(mode==2)
				{
					set_sp_Kp+=1;
				}
				else if(mode==4)
				{
					delayTime+=5;
				}
				break;}//K8 
              
	       case 13:{
				if(Stop_flag==1)
				{
					moveDelay=1;
					OLED_Init();
				}
				break;}//K9  延时启动 
              
          case 21: {
				if(mode==0)
				{
					loca_Kd_general=loca_Kd_general-5;
				}
				else if(mode==1)
				{
					Start_Center=Start_Center-1;
				}
				else if(mode==2)
				{
					set_sp_Ki-=0.2;
				}
				else if(mode==3)
				{
					debugTimer-=100;
				}
				break;}//k4 
              
	       case 22: {
				if(mode==0)
				{
					loca_Kd_general=loca_Kd_general+5;
				}
				else if(mode==1)
				{
					Start_Center=Start_Center+1;
				}
				else if(mode==2)
				{
					set_sp_Ki+=0.1;
				}
				else if(mode==3)
				{
					debugTimer+=100;
				}
				break;
			 }//k5
            
			 case 23:{
					OLED_Init();
				break; }//k6
            
		    case 31:{
				if(mode==0)
				{
					loca_Kp_general-=0.1;
				}
				else if(mode==1)
				{
					threshold-=5;
				}
				else if(mode==2)
				{
					set_speed-=10;
				}
				else if(mode==3)
				{
					set_sp_Kd-=1;
				}
				break;}//k1
            
      	 case 32:{
				if(mode==0)
				{
					loca_Kp_general+=0.2;
				}
				else if(mode==1)
				{
					threshold+=5;
				}
				else if(mode==2)
				{
					set_speed+=10;
				}
				else if(mode==3)
				{
					set_sp_Kd+=1;
				}
				
				break;}//k2 
            
			 case 33:{
				mode+=1;
				OLED_CLS();
				if(mode>4)
				  mode=0;
				break;} //k3 

		    default:break;

   }
}
