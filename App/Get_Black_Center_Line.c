#include "common.h"
#include "include.h"
#include "Get_Black_Center_Line.h"
/*图像用*/
extern uint8_t ADdata[DATALINE][DATACOUNT] ; //黑线AD数组存储（原始图像数据）
extern uint8_t Data[DATALINE][DATACOUNT];//二值化处理过后的图像数据
extern uint8_t threshold;//阈值
uint8_t  DMA_Over_Flag=0;//DMA搬运成功标志
//extern uint8_t  checkflag; //检查是否完成I2C写入配置标志位
/*处理赛道用*/
uint16 Black_Width=85;
int16 Black_Center_Last=55;
int Left_Line[20]={0};
int Right_Line[20]={0};
int Centerline[20]={0};
uint8 Start=39 ;//开始处理的行
uint8 End=20;//结束处理的行
uint8 Start_Center=32;//开始中心处理的行
uint8 End_Center=23;//结束中心处理的行
/*十字路口用*/
//uint8 Lose_Both_Count;  //用于丢线判断十字
uint8 Cross_flag=0;
uint16 Black_Left_Last;
uint16 Black_Right_Last;
//uint8 cnt=1;
/*舵机用 */
int Black_Center=55;
extern uint32 Servo_Duty;
float Servo_Err=0;
float Servo_Err_Last=0;
float loca_Kp;
float loca_Kd;
float loca_Kp_general=6.0;
float loca_Kd_general=25.0;
extern uint32 Servo_Duty_median;//舵机中值
float slope;//斜率
uint16 delayTime=150;//避障用，舵机打角延时
/*拨码开关状态标志位*/
extern uint8 Switch2_State;
extern uint8 Switch3_State;
extern uint8 Switch4_State;
extern uint8 Switch5_State;
/*电机开环用*/
//extern uint32 Motor_Duty;
/*测时用*/
//extern uint32 time_vlaue;
/*OLED显示赛道用*/
extern uint8 tempimg[40][128];
/*上位机看波形用*/
uint32 send_buff[2];
extern struct pid sPID;
extern float set_speed;
/*障碍处理基于前瞻24.5cm*/
/*   行   停车线黑线宽度    障碍黑线宽度     总赛道宽度   障碍内侧离中心线距离
     24        18                 9               41               4
     25        20                10               43               4
     26        22                11               47               4
     27        24                12               49               5
     28        24                12               51               4
     29        26                13               55               4
     30        26                13               57               4
     31        28                14               60               4
     32        28                14               63               5
*/
float t;//步长
int obstacleRow=26;
int obstacle[2]={0};
int obstacleCenter=0;
uint8 obstacleLeft_flag=0;
uint8 obstacleRight_flag=0;
uint8 obstacle_flag=0; //这一部分用于在线调试时将参数变为全局变量，可定义在子函数里面形成局部变量。
	 
	   	 
/*声明*/
void Search_Line(void);

void Get_IMG()
{
    if(DMA_Over_Flag == 1)
     {   
		  DMA_Over_Flag = 0 ;
		  port_init (PTA1,  GPI_DISAB| PF | ALT1 | PULLUP );         //关场中断
		  port_init (PTA6,  GPI_DISAB| PF | ALT1 | PULLUP );       //关行中断 
        Search_Line();//寻线
		  Servo_Duty=Servo_Control();
        tpm_pwm_duty(TPM2,TPM_CH1,Servo_Duty);
		  
		  
		  /*****发送图像到上位机*****/ 
        if(Switch2_State==1)         
        { 
          led_turn(LED0);
//			 gpio_set(PTB2,0);
		    uart_putchar(UART0,0xff);              
          for(int i=0;i<DATALINE;i++)
          {
            for(int j=0;j<DATACOUNT;j++)
            {
              if(ADdata[i][j]==0xff)
                {
                  ADdata[i][j]=0xfe;  
                 } //改变像素值防止误判为通讯结束
                 uart_putchar(UART0,ADdata[i][j]);
             }
          }
		  }
//		  if(Switch3_State==1)
//		  {
//			 /*****发送波形到上位机*****/ 
//			 send_buff[0]=(uint32)sPID.ActualSpeed;
//			 send_buff[1]=(uint32)set_speed;
//			 vcan_sendware((uint8*)send_buff,sizeof(send_buff));
//			 led_turn(LED2);
//		  }
		 /*EnableInterrupts所以这句开总中断也变得没必要了，下面有单独开场行中断*/
       port_init (PTA1,  IRQ_FALLING| PF | ALT1 | PULLUP );//开场中断,场中断函数中开行中断函数 
     }
}





void Search_Line(void)
{	 /*普通赛道用*/
  	 int Lose_Left_Line[19]={0};
    int Lose_Right_Line[19]={0};
	 uint32 Lose_Left_Standard=0;
	 uint32 Lose_Right_Standard=0;
	 uint16 Arithmetic_Average=0;
	 uint8 *p;
	 uint8 Lost_Left_count=0;
	 uint8 Lost_Right_count=0;
	 /*十字用*/
    uint16 Cross_Left;
	 uint16 Cross_Right;
	 uint8 Cross_Left_Lost;
	 uint8 Cross_Right_Lost;
	 uint16 Cross_Center;
    /*DisableInterrupts;这里一定不能关总中断！因为在这之前已经单独关掉场行中断了，关总中断会影响到定时中断！！！！*/
    Binaryzation();//用于数据处理简单二值化
	 if(Black_Width<=85) Black_Width=85;/*******************************2016.7.7晚上记得再看看，修改前瞻要改*************************************/
	 /*普通赛道*/
	 if(Cross_flag==0)
	 {
	   //基准行找左右基准边界
		Lose_Left_Standard=0;
		Lose_Right_Standard=0;
		for(int j=Black_Center<45?45:Black_Center;j>3;j--)//向左找左基准边界
		{    
			 if(Data[Start][j]==1&&Data[Start][j-1]==0)
			 {
				if(Data[Start][j+1]==1&&Data[Start][j-2]==0)
				{
				  Left_Line[Start-End]=j;//记录基准行左边界的列值
				  Lose_Left_Standard=0;//找到的标志位
				  break;//跳出循环
				}
			 }
			 else
			 {
				  Left_Line[Start-End]=15;//记基准行左边界的列值
				  Lose_Left_Standard=1;//找不到的标志位
				  continue;
			 }
			 
		}
		for(int j=Black_Center>65?65:Black_Center;j<106;j++)//向右找右基准边界
		{
			 if(Data[Start][j]==1&&Data[Start][j+1]==0)
			 {
				if(Data[Start][j-1]==1&&Data[Start][j+2]==0)
				{
				  Right_Line[Start-End]=j;//记录基准行右边界的列值
				  Lose_Right_Standard=0;//找到的标志位
				  break;//跳出循环
				}
			 }
			 else
			 {
				  Right_Line[Start-End]=95;//记录基准行右边界的列值
				  Lose_Right_Standard=1;//找不到标志位
				  continue;
			 }
		}
		if(Lose_Left_Standard==0&&Lose_Right_Standard==0)//两边基准都找到
		{
			  Centerline[Start-End]=(Left_Line[Start-End]+Right_Line[Start-End])/2;
			  Black_Width=Right_Line[Start-End]-Left_Line[Start-End];
//			  Centerline[Start-End]=(int)(Left_Line[Start-End]+Black_Width/2)>106?106:(Left_Line[Start-End]+Black_Width/2);
//			  Centerline[Start-End]=(int)(Right_Line[Start-End]-Black_Width/2)<3?3:(Right_Line[Start-End]-Black_Width/2);
  //			Lose_Both_Standard=0;
		}
		else if(Lose_Left_Standard==1&&Lose_Right_Standard==0)//左丢
		{
			  Centerline[Start-End]=(int)(Right_Line[Start-End]-Black_Width/2)<3?3:(Right_Line[Start-End]-Black_Width/2);
			  Left_Line[Start-End]=15;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!重要!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!不能如下语句
  //			Left_Line[Start-End]=3;//?????????????????  FWL diffrent
  //			Lose_Both_Standard=0;
		}
		else if(Lose_Left_Standard==0&&Lose_Right_Standard==1)//右丢
		{
			  Centerline[Start-End]=(int)(Left_Line[Start-End]+Black_Width/2)>106?106:(Left_Line[Start-End]+Black_Width/2);
			  Right_Line[Start-End]=95;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!重要!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!不能如下语句
  //			Right_Line[Start-End]=106;//????????????????   FWL diffrent
  //			Lose_Both_Standard=0
		}
		else
		{		
			Centerline[Start-End]=Black_Center;
       
		}
		gpio_set(PTB2,1);
	 }
		
	else//Cross_flag=1时候进不去找基准行函数，所以会找不到基准边界！
	{
//		Centerline[Start-End]=Black_Center;
//		Left_Line[Start-End]=Black_Left_Last;
//		Right_Line[Start-End]=Black_Right_Last;
		Centerline[Start-End]=Black_Center;
		Left_Line[Start-End]=15;
		Right_Line[Start-End]=95;
//		gpio_set(PTB2,0);
		
	}
		/**********************跟踪法找边界******************换图像的列值要改此段函数******************/
		for(int i=Start-1;i>=End;i--)
		{ 
		  //滤波
		  p=&Data[i][((Left_Line[i-(End-1)])-15)<3?3:((Left_Line[i-(End-1)])-15)];
		  while(p<&Data[i][((Left_Line[i-(End-1)])+15)>Centerline[i-End+1]?Centerline[i-End+1]:(Left_Line[i-(End-1)]+15)])
		  {
			 if(*(p-1)==1&&*p==0&&*(p+1)==1)		*p=1;
			 else if(*(p-1)==0&&*p==1&&*(p+1)==0)	*p=0;
			 p++;
		  }
		  //找左边界
		  for(int j=((Left_Line[i-(End-1)]+15)>106?106:(Left_Line[i-(End-1)]+15));j>((Left_Line[i-(End-1)]-15)<3?3:(Left_Line[i-(End-1)]-15));j--)//左边边界+10列到左边边界-10列的范围
		  {
			 if(Data[i][j]==1&&Data[i][j-1]==0)
			 {
				if(Data[i][j+1]==1&&Data[i][j-2]==0)
				{
				  Left_Line[i-End]=j;//记录第I行左边界的列值
				  Lose_Left_Line[i-End]=0;//Lose_Left_Line[8]
				  break;//跳出循环
				}
			 }
			 else
			 {
  //          Left_Line[i-End]=Left_Line[i-(End-1)];//记录第I行左边界的列值为上一次的列值 
				  Left_Line[i-End]=15;
				  Lose_Left_Line[i-End]=1;//Lose_Left_Line[8]
				  continue;
				  /*新想法
				  Left_Line[i-End]=常量;
				  Lose_Left_Line[i-End]=1;//Lose_Left_Line[8]
				  continue;
				  */
			 }
		  }
		  //滤波
		  p=&Data[i][((Right_Line[i-(End-1)])-15)>106?106:((Right_Line[i-(End-1)])-15)];
		  while(p<&Data[i][((Right_Line[i-(End-1)])+15)>Centerline[i-End+1]?Centerline[i-End+1]:(Right_Line[i-(End-1)]+15)])
		  {
			 if(*(p-1)==1&&*p==0&&*(p+1)==1)		*p=1;
			 else if(*(p-1)==0&&*p==1&&*(p+1)==0)	*p=0;
			 p++;
		  }
		  //找右边界
		  for(int j=((Right_Line[i-(End-1)]-15)<3?3:(Right_Line[i-(End-1)]-15));j<((Right_Line[i-(End-1)]+15)>106?106:(Right_Line[i-(End-1)]+15));j++)//右边边界-10列到右边边界+10列的范围
		  {
			 if(Data[i][j]==1&&Data[i][j+1]==0)
			 {
				if(Data[i][j-1]==1&&Data[i][j+2]==0)
				{
				  Right_Line[i-End]=j;//记录第I行右边界的列值
				  Lose_Right_Line[i-End]=0;//Lose_Right_Line[8]
				  break;//跳出循环
				}
			 }
			 else
			 {
  //            Right_Line[i-End]=Right_Line[i-(End-1)];//记录第I行右边界的列值为上一次的列值 
				  Right_Line[i-End]=95;
				  Lose_Right_Line[i-End]=1;//Lose_Right_Line[8]
				  continue;
				  /*新想法
				  Right_Line[i-End]=常量;
				  Lose_Right_Line[i-End]=1;//Lose_Right_Line[8]
				  continue;
				  */
			 }
		  }
		  

		  if(Lose_Left_Line[i-End]==0&&Lose_Right_Line[i-End]==0)//没有丢线
		  {
			  Centerline[i-End]=(Left_Line[i-End]+Right_Line[i-End])/2;
			  Black_Width=Right_Line[i-End]-Left_Line[i-End];
			  Centerline[i-End]=(int)(Left_Line[i-End]+Black_Width/2)>106?106:(Left_Line[i-End]+Black_Width/2);
			  Centerline[i-End]=(int)(Right_Line[i-End]-Black_Width/2)<3?3:(Right_Line[i-End]-Black_Width/2);		/***********可以试试109，1这样，尽量拓宽范围，不知道有没有影响到其他***********/			
		  }
		  else if(Lose_Left_Line[i-End]==1&&Lose_Right_Line[i-End]==0)//左丢
		  {
			  Centerline[i-End]=(int)(Right_Line[i-End]-Black_Width/2)<3?3:(Right_Line[i-End]-Black_Width/2);
  //			Left_Line[i-End]=3;//?????????????????
			  Left_Line[i-End]=Left_Line[i-End+1];//?????????????????	
			  Lost_Left_count++;
		  }
		  else if(Lose_Left_Line[i-End]==0&&Lose_Right_Line[i-End]==1)//右丢
		  {
			  Centerline[i-End]=(int)(Left_Line[i-End]+Black_Width/2)>106?106:(Left_Line[i-End]+Black_Width/2);
  //			Right_Line[i-End]=107;//????????????????
			  Right_Line[i-End]=Right_Line[i-End+1];//????????????????
			   Lost_Right_count++;
		  }
		  else
		  {
			  Centerline[i-End]=Centerline[i-End+1];	
			  Left_Line[i-End]=Left_Line[i-End+1];
			  Right_Line[i-End]=Right_Line[i-End+1];
			  /*此处用于丢边界判断进入十字----->不好处理，有时候转弯时会误判进入十字。*/
//			  if(i>=Start-15&&i<=(Start-3))      
//			  {		
//               Lose_Both_Count++;
//            }  
		  }
//		  if(Lose_Both_Count>=4)	//如果Lose_Both_Count没有>=3,结尾要清零！！重要！！ 
//		  {
//     		  //gpio_set (PTB2, 0);
//			  Lose_Both_Count=0;
//			  Cross_flag=1;
//		  }
		}
	  //障碍处理
	   if(Switch5_State==1)
		{
		   Obstacle_Handle();
		}
	   /**************************Cross_flag=1时候进不去函数找基准行的时候，赋给基准左右边界各一个值，取常量的时候这两句话没用了****************************/
	   Black_Left_Last=Left_Line[10];//但是last 要取哪个值？！！！！如果是距离最近的呢？赋予找不到的起始行
		Black_Right_Last=Right_Line[10];//last 用于Cross_flag=1时候进不去函数找基准行的时候，赋给基准左右边界各一个值。以便于跟踪法找边界能用。
	   
		//中心线的点数据处理
		slope=Slope(Start_Center,End_Center,Centerline);
		Arithmetic_Average=0;  
		for(int i=End_Center;i<=Start_Center;i++)
      {
       	Arithmetic_Average+=Centerline[i-End];
      }
//	   Black_Center=(int16)(Arithmetic_Average/10-slope);
		Black_Center=(int16)(Arithmetic_Average/10);
		if(obstacleLeft_flag==1)
		{
//		   obstacleLeft_flag=0;
			t=fabs(Black_Center-obstacleCenter);
			Black_Center=(int)(Black_Center+16);
//		  Black_Center=obstacleCenter;
//		  
		}
		else if(obstacleRight_flag==1)
		{
//		   obstacleRight_flag=0;
			t=fabs(obstacleCenter-Black_Center);
			Black_Center=(int)(Black_Center-13);
//		  Black_Center=obstacleCenter;
		}
      Black_Center_Last= Black_Center;
		
		/*十字处理*/ 
		/*十字处理思想：一判断进入十字就能扫到对面有边界的那一行，从而跟踪其中心*/
		if(Cross_flag==1)
		{    
//		    gpio_set(PTB2,0);
			 led_turn(LED1);
			 for(int i=20;i>19;i--)
			 {
				 for(int j=65;j>=2;j--)//左
				 {
//					 if(ADdata[i][j+2]>threshold && ADdata[i][j+1]>threshold && ADdata[i][j]>threshold && ADdata[i][j-1]<threshold && ADdata[i][j-2]<threshold)
					 if(Data[i][j+2]==1&&Data[i][j+1]==1&&Data[i][j]==1&&Data[i][j-1]==0&&Data[i][j-2]==0)
					 {
						 Cross_Left=j;
						 Cross_Left_Lost=0;
						 break;
					 }
					 else
					 {
						 Cross_Left=3;
						 Cross_Left_Lost=1;
						 continue;
					 }
				 }
				 for(int j=45;j<=106;j++)//右
				 {
//					 if(ADdata[i][j+2]<threshold && ADdata[i][j+1]<threshold && ADdata[i][j]>threshold && ADdata[i][j-1]>threshold && ADdata[i][j-2]>threshold)
					 if(Data[i][j+2]==0&&Data[i][j+1]==0&&Data[i][j]==1&&Data[i][j-1]==1&&Data[i][j-2]==1)
					 {
						 Cross_Right=j;
						 Cross_Right_Lost=0;
						 break;
					 }
					 else
					 {
						 Cross_Right=106;
						 Cross_Right_Lost=1;
						 continue;
					 }
				 }
			 }
			 if(Cross_Left_Lost==0&&Cross_Right_Lost==0)
			 {
				 Cross_Center=(int)((Cross_Left+Cross_Right)/2);
				 Cross_flag=0;
				 gpio_set(PTB2,0);
			 }
			 else 
			 {
				Cross_Center=Black_Center_Last;//
//				gpio_set(PTB2,1);
                             
			 }
			 Black_Center=Cross_Center;
			 Black_Center_Last=Black_Center;
			 
		}
// 	  Lose_Both_Count=0;

}		

void IMG_Init()
{	
	uint8_t i,j;
	for(i=0;i<DATALINE;i++)
	      for(j=0;j<DATACOUNT;j++)
	        ADdata[i][j]=0;
}


/*前车车轮中心到39行的距离：26cm
前车车轮中心到21行的距离：99cm
右转：参数：33~24行，err<17，err>0时KP=12;   err>17,err>0,KP=12+9------KD=10
左转：参数：33~24行，err<-17，err《0时KP=10;   err>-17,err《0,KP=10+9------KD=10*/
int16 Servo_Control(void)
{
    /*PD分二段,优点，有连续性*/
	 Servo_Err=(float)(Black_Center-Black_Center_Theory);
	 if(Servo_Err<17&&Servo_Err>0) //右
	 {
		loca_Kp=19;
		loca_Kd=40;
//		gpio_set(PTB2,0);
	 }
	 else if (Servo_Err>=17&&Servo_Err>0)  
	 {
		loca_Kp=19+loca_Kp_general;
		loca_Kd=40+loca_Kd_general;
//		gpio_set(PTB2,1);
	 }
	 else if(Servo_Err>-17&&Servo_Err<0)//左
	 {
	 	loca_Kp=13;
		loca_Kd=40;
	 }
	 else if(Servo_Err<-17&&Servo_Err<0)
	 {
	 	loca_Kp=13+loca_Kp_general;
		loca_Kd=40+loca_Kd_general;
	 }
	 
	 if(obstacleLeft_flag==1)//障碍在左边
	 {
	 	 loca_Kp=18;
		 loca_Kd=35;
	 }
	 if(obstacleRight_flag==1)//障碍在右边
	 {
	 	 loca_Kp=22;
		 loca_Kd=35;
	 }
	 
//	 loca_Kp=loca_Kp_general;
	 
    Servo_Duty=(int)(loca_Kp*Servo_Err+loca_Kd*(Servo_Err-Servo_Err_Last));
//	 Servo_Duty=(int)(loca_Kp*Servo_Err);
	 Servo_Err_Last=Servo_Err;
	 Servo_Duty =Servo_Duty_median+Servo_Duty;

	 //舵机限幅（中值4520）
	 if(Servo_Duty>(Servo_Duty_median+1200)) {Servo_Duty=(Servo_Duty_median+1200);}
	 if(Servo_Duty<(Servo_Duty_median-1200)) {Servo_Duty=(Servo_Duty_median-1200);}  
    return (int32)Servo_Duty;
}

/*最小二乘法求斜率*/
float Slope(const uint8 start_c,const uint8 end_c,const int * C_L) //右转为负，左转为正
{
   //最小二乘法所需参数 
    float SumX=0;
    float SumX2=0;
    float SumY=0;
    float SumXY=0;
	 float Slope_k;
	 static float Slope_k_Last;

    for(int i=end_c;i<=start_c;i++)
    {
        SumX+=i;
        SumX2+=i*i;
        SumY+=*(C_L+i-End);
        SumXY+=i*(*(C_L+i-End));
    }
	 if((start_c-end_c)*SumX2-SumX*SumX)
	 {
    	Slope_k=(float)(((start_c-end_c+1)*SumXY-SumX*SumY)/((start_c-end_c+1)*SumX2-SumX*SumX));
		Slope_k_Last=Slope_k;
	 }
	 else Slope_k=Slope_k_Last;
 
	 return Slope_k;
}


int White_line=0;
int White_count=0;
//int outStop_flag=0;
void Binaryzation(void)  //用于数据处理简单二值化
{
  uint8_t i,j;
//  uint8 Black_count=0;
  White_line=0;
  for(i=End;i<=Start;i++)
  {
 // 		Data[i][Centerline[i-End]]=0;
		ADdata[i][Centerline[i-End]]=0;
  }
  
  for(i=Start;i>=End;i--)   //要处理的行数,其中多处理的行为十字用
  { 
	   for(j=0;j<DATACOUNT;j++)
		{
		  if(ADdata[i][j]>threshold)
		  {
				Data[i][j]=1;
				White_count++;
		  } //大于阈值则为白
		  else
		  { 
			  Data[i][j]=0;  
//			  Black_count++;//小于阈值则为黑
		  }
		  if(j>=4&&j<=124)//用于将采到的图像映射到OLED显示
		  {
			  tempimg[i][j]=Data[i][j];
		  }
		}
	  if(i<=35)
	  {
		  if(White_count>108)//全白行计数
		  {
				White_count=0;
				White_line++;
		  }
		  else{ White_count=0;}
	   }
	   else {White_count=0;}
		
//		if(i==35)
//		{
//			if(Black_count>109)
//			{
//				Black_count=0;
//				outStop_flag=1;
//			}
//		}
	  

   }
  if(White_line>3)
  {
		 Cross_flag=1;
		 White_line=0;
		
  }
  else 
  {
  		White_line=0;
  }
}

/*障碍处理程序*/
void Obstacle_Handle(void)
{
	for(int j=60;j>3;j--)//往左
	{
		if(Data[obstacleRow][j-2]==0&&Data[obstacleRow][j-1]==0&&Data[obstacleRow][j]==1&&Data[obstacleRow][j+1]==1)
		{
			obstacle[0]=j;
			break;
		}
		else  obstacle[0]=3;
				
	}
	for(int j=50;j<106;j++)//往右
	{
		if(Data[obstacleRow][j-1]==1&&Data[obstacleRow][j]==1&&Data[obstacleRow][j+1]==0&&Data[obstacleRow][j+2]==0)
		{
			obstacle[1]=j;
			break;
		}
		else  obstacle[1]=106;
	}
			
	if((obstacle[1]-obstacle[0])>25&&(obstacle[1]-obstacle[0])<35)
	{
		obstacleCenter=(obstacle[1]+obstacle[0])/2;
		if(obstacle[0]!=3&&obstacle[1]!=106&&obstacleCenter<55)
		{
			  if(Data[obstacleRow][obstacle[1]+1]==0&&Data[obstacleRow][obstacle[1]+2]==0&&Data[obstacleRow][obstacle[1]+4]==0)
			  {
				 if(Data[obstacleRow-2][obstacle[1]+2]==0&&Data[obstacleRow-2][obstacle[1]+3]==0&&Data[obstacleRow-2][obstacle[1]+4]==0)//防止因为图像原因把停车线误差为障碍
				 {
					 obstacleRight_flag=1;
					 obstacle_flag=1;
					 gpio_set(PTB2,0);
					 pit_init_ms(PIT1,delayTime); //适当延时使中心相应偏移
				 }
			  }
		}
		else if(obstacle[0]!=3&&obstacle[1]!=106&&obstacleCenter>55)
		{
			if(Data[obstacleRow][obstacle[0]-1]==0&&Data[obstacleRow][obstacle[0]-2]==0&&Data[obstacleRow][obstacle[0]-4]==0)
			{
			  if(Data[obstacleRow-2][obstacle[0]-2]==0&&Data[obstacleRow-2][obstacle[0]-3]==0&&Data[obstacleRow-2][obstacle[0]-4]==0)//防止因为图像原因把停车线误差为障碍
			  {
				  obstacleLeft_flag=1;
				  obstacle_flag=1;
				  gpio_set(PTB2,0);
				  pit_init_ms(PIT1,delayTime); //适当延时使中心相应偏移
			  }
			}
		}
		obstacle[0]=obstacle[1]=0;
		
	}
	else 
	{
		obstacle[0]=obstacle[1]=0;
//		obstacleLeft_flag=obstacleRight_flag=0;
//	   obstacle_flag=0;
//		gpio_set(PTB2,0);
	}
//	obstacleLeft_flag=obstacleRight_flag=0;
//	obstacle[0]=obstacle[1]=0;
}