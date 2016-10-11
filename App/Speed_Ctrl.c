#include "common.h"
#include "include.h"
#include "Speed_Ctrl.h"
/*���i4.3��p40��d3 */ 
uint32 Speed_Duty;
struct pid sPID;
uint8 statetemp1=1;
extern uint8 Stop_flag;
//extern uint8 slow_flag;
extern float slope;
float set_speed=200;
double set_sp_Kp=40;
float set_sp_Ki=4.3;
double set_sp_Kd=3;
/*���PID������ʼ��*/
void PID_Init(void)
{
	sPID.SetSpeed=0.0;
	sPID.ActualSpeed=0.0;
	sPID.MotorDuty=0.0;
	sPID.err[0]=0.0;
	sPID.err[1]=0.0;
	sPID.err[2]=0.0;
	sPID.Kp=0.0;
	sPID.Ki=0.0;
	sPID.Kd=0.0;
}
/*���PID���������޸�*/
void PID_Fix(void)
{
//	if(slope>1)
//	{
//   	sPID.SetSpeed=set_speed-30;
//	}
//	else sPID.SetSpeed=set_speed;
   sPID.SetSpeed=set_speed;
	sPID.Kp=0+set_sp_Kp;
	sPID.Ki=0+set_sp_Ki;
	sPID.Kd=0+set_sp_Kd;
//	sPID.Kd=0.0;
}
/*���PID����*/
float PID_Realize(struct pid *p)
{
   float incrementSpeed=0;
	float index=0;
	float iSeparate;
	index=(1-0.05*fabs(slope))<=0.8?0.8:(1-0.05*fabs(slope));//Ҫ�ǵüӾ���ֵ��
	p->err[0]=p->SetSpeed*index-p->ActualSpeed;
//	if(p->err[0]>50)//���ַ��룬�������������ֵ��ƫ��ϴ�ʱ��ȥ�����֣�������ֱ��ͣ��������������ֵ�ӽ�ʱ��������֣�������̬���
//	  iSeparate=0;
//	else
//	  iSeparate=1;
	incrementSpeed=p->Kp*(p->err[0]-p->err[1])+p->Ki*p->err[0]+p->Kd*(p->err[0]-2*p->err[1]+p->err[2]);
	p->MotorDuty+=incrementSpeed;
	if(p->MotorDuty>=6000) p->MotorDuty=6000;
	if(p->MotorDuty<=0) p->MotorDuty=0;
	p->err[2]=p->err[1];
	p->err[1]=p->err[0];
	return p->MotorDuty;
}
/*����ٶȿ���*/
void SpeedCtrl(void)
{
	PID_Fix();
   Speed_Duty=(int)PID_Realize(&sPID);
	tpm_pwm_duty(TPM0, TPM_CH1, Speed_Duty);
   tpm_pwm_duty(TPM0, TPM_CH2, 0);
}
/*��תͣ��*/
void Motor_Stall(void)
{
	uint8 statetemp2=0;//��ת��־λ
	if(sPID.ActualSpeed>=100)
	  statetemp1=1;
	if(statetemp1==1)
	{
		if(sPID.ActualSpeed==0)
		{
			statetemp2=1;
		}
		else statetemp2=0;
	}
	if(statetemp2==1)//��ת��־λ
	{
		Stop_flag=1;
		statetemp1=0;
	}
}
/*ɽ������ʾ����*/
void vcan_sendware(uint8 *wareaddr, uint32 waresize)
{
    #define CMD_WARE   3
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //���ڵ��� ʹ�õ�ǰ����
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //���ڵ��� ʹ�õĺ�����

    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //�ȷ���ǰ����
    uart_putbuff(VCAN_PORT, wareaddr, waresize);    //��������
    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //���ͺ�����
}


