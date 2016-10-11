#include "common.h"
#include "include.h"
#include "Get_Black_Center_Line.h"
/*ͼ����*/
extern uint8_t ADdata[DATALINE][DATACOUNT] ; //����AD����洢��ԭʼͼ�����ݣ�
extern uint8_t Data[DATALINE][DATACOUNT];//��ֵ����������ͼ������
extern uint8_t threshold;//��ֵ
uint8_t  DMA_Over_Flag=0;//DMA���˳ɹ���־
//extern uint8_t  checkflag; //����Ƿ����I2Cд�����ñ�־λ
/*����������*/
uint16 Black_Width=85;
int16 Black_Center_Last=55;
int Left_Line[20]={0};
int Right_Line[20]={0};
int Centerline[20]={0};
uint8 Start=39 ;//��ʼ�������
uint8 End=20;//�����������
uint8 Start_Center=32;//��ʼ���Ĵ������
uint8 End_Center=23;//�������Ĵ������
/*ʮ��·����*/
//uint8 Lose_Both_Count;  //���ڶ����ж�ʮ��
uint8 Cross_flag=0;
uint16 Black_Left_Last;
uint16 Black_Right_Last;
//uint8 cnt=1;
/*����� */
int Black_Center=55;
extern uint32 Servo_Duty;
float Servo_Err=0;
float Servo_Err_Last=0;
float loca_Kp;
float loca_Kd;
float loca_Kp_general=6.0;
float loca_Kd_general=25.0;
extern uint32 Servo_Duty_median;//�����ֵ
float slope;//б��
uint16 delayTime=150;//�����ã���������ʱ
/*���뿪��״̬��־λ*/
extern uint8 Switch2_State;
extern uint8 Switch3_State;
extern uint8 Switch4_State;
extern uint8 Switch5_State;
/*���������*/
//extern uint32 Motor_Duty;
/*��ʱ��*/
//extern uint32 time_vlaue;
/*OLED��ʾ������*/
extern uint8 tempimg[40][128];
/*��λ����������*/
uint32 send_buff[2];
extern struct pid sPID;
extern float set_speed;
/*�ϰ��������ǰհ24.5cm*/
/*   ��   ͣ���ߺ��߿��    �ϰ����߿��     ���������   �ϰ��ڲ��������߾���
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
float t;//����
int obstacleRow=26;
int obstacle[2]={0};
int obstacleCenter=0;
uint8 obstacleLeft_flag=0;
uint8 obstacleRight_flag=0;
uint8 obstacle_flag=0; //��һ�����������ߵ���ʱ��������Ϊȫ�ֱ������ɶ������Ӻ��������γɾֲ�������
	 
	   	 
/*����*/
void Search_Line(void);

void Get_IMG()
{
    if(DMA_Over_Flag == 1)
     {   
		  DMA_Over_Flag = 0 ;
		  port_init (PTA1,  GPI_DISAB| PF | ALT1 | PULLUP );         //�س��ж�
		  port_init (PTA6,  GPI_DISAB| PF | ALT1 | PULLUP );       //�����ж� 
        Search_Line();//Ѱ��
		  Servo_Duty=Servo_Control();
        tpm_pwm_duty(TPM2,TPM_CH1,Servo_Duty);
		  
		  
		  /*****����ͼ����λ��*****/ 
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
                 } //�ı�����ֵ��ֹ����ΪͨѶ����
                 uart_putchar(UART0,ADdata[i][j]);
             }
          }
		  }
//		  if(Switch3_State==1)
//		  {
//			 /*****���Ͳ��ε���λ��*****/ 
//			 send_buff[0]=(uint32)sPID.ActualSpeed;
//			 send_buff[1]=(uint32)set_speed;
//			 vcan_sendware((uint8*)send_buff,sizeof(send_buff));
//			 led_turn(LED2);
//		  }
		 /*EnableInterrupts������俪���ж�Ҳ���û��Ҫ�ˣ������е����������ж�*/
       port_init (PTA1,  IRQ_FALLING| PF | ALT1 | PULLUP );//�����ж�,���жϺ����п����жϺ��� 
     }
}





void Search_Line(void)
{	 /*��ͨ������*/
  	 int Lose_Left_Line[19]={0};
    int Lose_Right_Line[19]={0};
	 uint32 Lose_Left_Standard=0;
	 uint32 Lose_Right_Standard=0;
	 uint16 Arithmetic_Average=0;
	 uint8 *p;
	 uint8 Lost_Left_count=0;
	 uint8 Lost_Right_count=0;
	 /*ʮ����*/
    uint16 Cross_Left;
	 uint16 Cross_Right;
	 uint8 Cross_Left_Lost;
	 uint8 Cross_Right_Lost;
	 uint16 Cross_Center;
    /*DisableInterrupts;����һ�����ܹ����жϣ���Ϊ����֮ǰ�Ѿ������ص������ж��ˣ������жϻ�Ӱ�쵽��ʱ�жϣ�������*/
    Binaryzation();//�������ݴ���򵥶�ֵ��
	 if(Black_Width<=85) Black_Width=85;/*******************************2016.7.7���ϼǵ��ٿ������޸�ǰհҪ��*************************************/
	 /*��ͨ����*/
	 if(Cross_flag==0)
	 {
	   //��׼�������һ�׼�߽�
		Lose_Left_Standard=0;
		Lose_Right_Standard=0;
		for(int j=Black_Center<45?45:Black_Center;j>3;j--)//���������׼�߽�
		{    
			 if(Data[Start][j]==1&&Data[Start][j-1]==0)
			 {
				if(Data[Start][j+1]==1&&Data[Start][j-2]==0)
				{
				  Left_Line[Start-End]=j;//��¼��׼����߽����ֵ
				  Lose_Left_Standard=0;//�ҵ��ı�־λ
				  break;//����ѭ��
				}
			 }
			 else
			 {
				  Left_Line[Start-End]=15;//�ǻ�׼����߽����ֵ
				  Lose_Left_Standard=1;//�Ҳ����ı�־λ
				  continue;
			 }
			 
		}
		for(int j=Black_Center>65?65:Black_Center;j<106;j++)//�������һ�׼�߽�
		{
			 if(Data[Start][j]==1&&Data[Start][j+1]==0)
			 {
				if(Data[Start][j-1]==1&&Data[Start][j+2]==0)
				{
				  Right_Line[Start-End]=j;//��¼��׼���ұ߽����ֵ
				  Lose_Right_Standard=0;//�ҵ��ı�־λ
				  break;//����ѭ��
				}
			 }
			 else
			 {
				  Right_Line[Start-End]=95;//��¼��׼���ұ߽����ֵ
				  Lose_Right_Standard=1;//�Ҳ�����־λ
				  continue;
			 }
		}
		if(Lose_Left_Standard==0&&Lose_Right_Standard==0)//���߻�׼���ҵ�
		{
			  Centerline[Start-End]=(Left_Line[Start-End]+Right_Line[Start-End])/2;
			  Black_Width=Right_Line[Start-End]-Left_Line[Start-End];
//			  Centerline[Start-End]=(int)(Left_Line[Start-End]+Black_Width/2)>106?106:(Left_Line[Start-End]+Black_Width/2);
//			  Centerline[Start-End]=(int)(Right_Line[Start-End]-Black_Width/2)<3?3:(Right_Line[Start-End]-Black_Width/2);
  //			Lose_Both_Standard=0;
		}
		else if(Lose_Left_Standard==1&&Lose_Right_Standard==0)//��
		{
			  Centerline[Start-End]=(int)(Right_Line[Start-End]-Black_Width/2)<3?3:(Right_Line[Start-End]-Black_Width/2);
			  Left_Line[Start-End]=15;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!��Ҫ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!�����������
  //			Left_Line[Start-End]=3;//?????????????????  FWL diffrent
  //			Lose_Both_Standard=0;
		}
		else if(Lose_Left_Standard==0&&Lose_Right_Standard==1)//�Ҷ�
		{
			  Centerline[Start-End]=(int)(Left_Line[Start-End]+Black_Width/2)>106?106:(Left_Line[Start-End]+Black_Width/2);
			  Right_Line[Start-End]=95;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!��Ҫ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!�����������
  //			Right_Line[Start-End]=106;//????????????????   FWL diffrent
  //			Lose_Both_Standard=0
		}
		else
		{		
			Centerline[Start-End]=Black_Center;
       
		}
		gpio_set(PTB2,1);
	 }
		
	else//Cross_flag=1ʱ�����ȥ�һ�׼�к��������Ի��Ҳ�����׼�߽磡
	{
//		Centerline[Start-End]=Black_Center;
//		Left_Line[Start-End]=Black_Left_Last;
//		Right_Line[Start-End]=Black_Right_Last;
		Centerline[Start-End]=Black_Center;
		Left_Line[Start-End]=15;
		Right_Line[Start-End]=95;
//		gpio_set(PTB2,0);
		
	}
		/**********************���ٷ��ұ߽�******************��ͼ�����ֵҪ�Ĵ˶κ���******************/
		for(int i=Start-1;i>=End;i--)
		{ 
		  //�˲�
		  p=&Data[i][((Left_Line[i-(End-1)])-15)<3?3:((Left_Line[i-(End-1)])-15)];
		  while(p<&Data[i][((Left_Line[i-(End-1)])+15)>Centerline[i-End+1]?Centerline[i-End+1]:(Left_Line[i-(End-1)]+15)])
		  {
			 if(*(p-1)==1&&*p==0&&*(p+1)==1)		*p=1;
			 else if(*(p-1)==0&&*p==1&&*(p+1)==0)	*p=0;
			 p++;
		  }
		  //����߽�
		  for(int j=((Left_Line[i-(End-1)]+15)>106?106:(Left_Line[i-(End-1)]+15));j>((Left_Line[i-(End-1)]-15)<3?3:(Left_Line[i-(End-1)]-15));j--)//��߽߱�+10�е���߽߱�-10�еķ�Χ
		  {
			 if(Data[i][j]==1&&Data[i][j-1]==0)
			 {
				if(Data[i][j+1]==1&&Data[i][j-2]==0)
				{
				  Left_Line[i-End]=j;//��¼��I����߽����ֵ
				  Lose_Left_Line[i-End]=0;//Lose_Left_Line[8]
				  break;//����ѭ��
				}
			 }
			 else
			 {
  //          Left_Line[i-End]=Left_Line[i-(End-1)];//��¼��I����߽����ֵΪ��һ�ε���ֵ 
				  Left_Line[i-End]=15;
				  Lose_Left_Line[i-End]=1;//Lose_Left_Line[8]
				  continue;
				  /*���뷨
				  Left_Line[i-End]=����;
				  Lose_Left_Line[i-End]=1;//Lose_Left_Line[8]
				  continue;
				  */
			 }
		  }
		  //�˲�
		  p=&Data[i][((Right_Line[i-(End-1)])-15)>106?106:((Right_Line[i-(End-1)])-15)];
		  while(p<&Data[i][((Right_Line[i-(End-1)])+15)>Centerline[i-End+1]?Centerline[i-End+1]:(Right_Line[i-(End-1)]+15)])
		  {
			 if(*(p-1)==1&&*p==0&&*(p+1)==1)		*p=1;
			 else if(*(p-1)==0&&*p==1&&*(p+1)==0)	*p=0;
			 p++;
		  }
		  //���ұ߽�
		  for(int j=((Right_Line[i-(End-1)]-15)<3?3:(Right_Line[i-(End-1)]-15));j<((Right_Line[i-(End-1)]+15)>106?106:(Right_Line[i-(End-1)]+15));j++)//�ұ߽߱�-10�е��ұ߽߱�+10�еķ�Χ
		  {
			 if(Data[i][j]==1&&Data[i][j+1]==0)
			 {
				if(Data[i][j-1]==1&&Data[i][j+2]==0)
				{
				  Right_Line[i-End]=j;//��¼��I���ұ߽����ֵ
				  Lose_Right_Line[i-End]=0;//Lose_Right_Line[8]
				  break;//����ѭ��
				}
			 }
			 else
			 {
  //            Right_Line[i-End]=Right_Line[i-(End-1)];//��¼��I���ұ߽����ֵΪ��һ�ε���ֵ 
				  Right_Line[i-End]=95;
				  Lose_Right_Line[i-End]=1;//Lose_Right_Line[8]
				  continue;
				  /*���뷨
				  Right_Line[i-End]=����;
				  Lose_Right_Line[i-End]=1;//Lose_Right_Line[8]
				  continue;
				  */
			 }
		  }
		  

		  if(Lose_Left_Line[i-End]==0&&Lose_Right_Line[i-End]==0)//û�ж���
		  {
			  Centerline[i-End]=(Left_Line[i-End]+Right_Line[i-End])/2;
			  Black_Width=Right_Line[i-End]-Left_Line[i-End];
			  Centerline[i-End]=(int)(Left_Line[i-End]+Black_Width/2)>106?106:(Left_Line[i-End]+Black_Width/2);
			  Centerline[i-End]=(int)(Right_Line[i-End]-Black_Width/2)<3?3:(Right_Line[i-End]-Black_Width/2);		/***********��������109��1�����������ؿ�Χ����֪����û��Ӱ�쵽����***********/			
		  }
		  else if(Lose_Left_Line[i-End]==1&&Lose_Right_Line[i-End]==0)//��
		  {
			  Centerline[i-End]=(int)(Right_Line[i-End]-Black_Width/2)<3?3:(Right_Line[i-End]-Black_Width/2);
  //			Left_Line[i-End]=3;//?????????????????
			  Left_Line[i-End]=Left_Line[i-End+1];//?????????????????	
			  Lost_Left_count++;
		  }
		  else if(Lose_Left_Line[i-End]==0&&Lose_Right_Line[i-End]==1)//�Ҷ�
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
			  /*�˴����ڶ��߽��жϽ���ʮ��----->���ô�����ʱ��ת��ʱ�����н���ʮ�֡�*/
//			  if(i>=Start-15&&i<=(Start-3))      
//			  {		
//               Lose_Both_Count++;
//            }  
		  }
//		  if(Lose_Both_Count>=4)	//���Lose_Both_Countû��>=3,��βҪ���㣡����Ҫ���� 
//		  {
//     		  //gpio_set (PTB2, 0);
//			  Lose_Both_Count=0;
//			  Cross_flag=1;
//		  }
		}
	  //�ϰ�����
	   if(Switch5_State==1)
		{
		   Obstacle_Handle();
		}
	   /**************************Cross_flag=1ʱ�����ȥ�����һ�׼�е�ʱ�򣬸�����׼���ұ߽��һ��ֵ��ȡ������ʱ�������仰û����****************************/
	   Black_Left_Last=Left_Line[10];//����last Ҫȡ�ĸ�ֵ��������������Ǿ���������أ������Ҳ�������ʼ��
		Black_Right_Last=Right_Line[10];//last ����Cross_flag=1ʱ�����ȥ�����һ�׼�е�ʱ�򣬸�����׼���ұ߽��һ��ֵ���Ա��ڸ��ٷ��ұ߽����á�
	   
		//�����ߵĵ����ݴ���
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
		
		/*ʮ�ִ���*/ 
		/*ʮ�ִ���˼�룺һ�жϽ���ʮ�־���ɨ�������б߽����һ�У��Ӷ�����������*/
		if(Cross_flag==1)
		{    
//		    gpio_set(PTB2,0);
			 led_turn(LED1);
			 for(int i=20;i>19;i--)
			 {
				 for(int j=65;j>=2;j--)//��
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
				 for(int j=45;j<=106;j++)//��
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


/*ǰ���������ĵ�39�еľ��룺26cm
ǰ���������ĵ�21�еľ��룺99cm
��ת��������33~24�У�err<17��err>0ʱKP=12;   err>17,err>0,KP=12+9------KD=10
��ת��������33~24�У�err<-17��err��0ʱKP=10;   err>-17,err��0,KP=10+9------KD=10*/
int16 Servo_Control(void)
{
    /*PD�ֶ���,�ŵ㣬��������*/
	 Servo_Err=(float)(Black_Center-Black_Center_Theory);
	 if(Servo_Err<17&&Servo_Err>0) //��
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
	 else if(Servo_Err>-17&&Servo_Err<0)//��
	 {
	 	loca_Kp=13;
		loca_Kd=40;
	 }
	 else if(Servo_Err<-17&&Servo_Err<0)
	 {
	 	loca_Kp=13+loca_Kp_general;
		loca_Kd=40+loca_Kd_general;
	 }
	 
	 if(obstacleLeft_flag==1)//�ϰ������
	 {
	 	 loca_Kp=18;
		 loca_Kd=35;
	 }
	 if(obstacleRight_flag==1)//�ϰ����ұ�
	 {
	 	 loca_Kp=22;
		 loca_Kd=35;
	 }
	 
//	 loca_Kp=loca_Kp_general;
	 
    Servo_Duty=(int)(loca_Kp*Servo_Err+loca_Kd*(Servo_Err-Servo_Err_Last));
//	 Servo_Duty=(int)(loca_Kp*Servo_Err);
	 Servo_Err_Last=Servo_Err;
	 Servo_Duty =Servo_Duty_median+Servo_Duty;

	 //����޷�����ֵ4520��
	 if(Servo_Duty>(Servo_Duty_median+1200)) {Servo_Duty=(Servo_Duty_median+1200);}
	 if(Servo_Duty<(Servo_Duty_median-1200)) {Servo_Duty=(Servo_Duty_median-1200);}  
    return (int32)Servo_Duty;
}

/*��С���˷���б��*/
float Slope(const uint8 start_c,const uint8 end_c,const int * C_L) //��תΪ������תΪ��
{
   //��С���˷�������� 
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
void Binaryzation(void)  //�������ݴ���򵥶�ֵ��
{
  uint8_t i,j;
//  uint8 Black_count=0;
  White_line=0;
  for(i=End;i<=Start;i++)
  {
 // 		Data[i][Centerline[i-End]]=0;
		ADdata[i][Centerline[i-End]]=0;
  }
  
  for(i=Start;i>=End;i--)   //Ҫ���������,���жദ�����Ϊʮ����
  { 
	   for(j=0;j<DATACOUNT;j++)
		{
		  if(ADdata[i][j]>threshold)
		  {
				Data[i][j]=1;
				White_count++;
		  } //������ֵ��Ϊ��
		  else
		  { 
			  Data[i][j]=0;  
//			  Black_count++;//С����ֵ��Ϊ��
		  }
		  if(j>=4&&j<=124)//���ڽ��ɵ���ͼ��ӳ�䵽OLED��ʾ
		  {
			  tempimg[i][j]=Data[i][j];
		  }
		}
	  if(i<=35)
	  {
		  if(White_count>108)//ȫ���м���
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

/*�ϰ��������*/
void Obstacle_Handle(void)
{
	for(int j=60;j>3;j--)//����
	{
		if(Data[obstacleRow][j-2]==0&&Data[obstacleRow][j-1]==0&&Data[obstacleRow][j]==1&&Data[obstacleRow][j+1]==1)
		{
			obstacle[0]=j;
			break;
		}
		else  obstacle[0]=3;
				
	}
	for(int j=50;j<106;j++)//����
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
				 if(Data[obstacleRow-2][obstacle[1]+2]==0&&Data[obstacleRow-2][obstacle[1]+3]==0&&Data[obstacleRow-2][obstacle[1]+4]==0)//��ֹ��Ϊͼ��ԭ���ͣ�������Ϊ�ϰ�
				 {
					 obstacleRight_flag=1;
					 obstacle_flag=1;
					 gpio_set(PTB2,0);
					 pit_init_ms(PIT1,delayTime); //�ʵ���ʱʹ������Ӧƫ��
				 }
			  }
		}
		else if(obstacle[0]!=3&&obstacle[1]!=106&&obstacleCenter>55)
		{
			if(Data[obstacleRow][obstacle[0]-1]==0&&Data[obstacleRow][obstacle[0]-2]==0&&Data[obstacleRow][obstacle[0]-4]==0)
			{
			  if(Data[obstacleRow-2][obstacle[0]-2]==0&&Data[obstacleRow-2][obstacle[0]-3]==0&&Data[obstacleRow-2][obstacle[0]-4]==0)//��ֹ��Ϊͼ��ԭ���ͣ�������Ϊ�ϰ�
			  {
				  obstacleLeft_flag=1;
				  obstacle_flag=1;
				  gpio_set(PTB2,0);
				  pit_init_ms(PIT1,delayTime); //�ʵ���ʱʹ������Ӧƫ��
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