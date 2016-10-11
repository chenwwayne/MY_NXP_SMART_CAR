#include "common.h"
#include "include.h"
#include "Speed_Ctrl.h"
/*电机i4.3，p40，d3 */ 
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
/*电机PID参数初始化*/
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
/*电机PID参数按键修改*/
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
/*电机PID控制*/
float PID_Realize(struct pid *p)
{
   float incrementSpeed=0;
	float index=0;
	float iSeparate;
	index=(1-0.05*fabs(slope))<=0.8?0.8:(1-0.05*fabs(slope));//要记得加绝对值！
	p->err[0]=p->SetSpeed*index-p->ActualSpeed;
//	if(p->err[0]>50)//积分分离，当被控量与给定值的偏差较大时，去掉积分，避免积分饱和；当被控量与给定值接近时，引入积分，消除静态误差
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
/*电机速度控制*/
void SpeedCtrl(void)
{
	PID_Fix();
   Speed_Duty=(int)PID_Realize(&sPID);
	tpm_pwm_duty(TPM0, TPM_CH1, Speed_Duty);
   tpm_pwm_duty(TPM0, TPM_CH2, 0);
}
/*堵转停车*/
void Motor_Stall(void)
{
	uint8 statetemp2=0;//堵转标志位
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
	if(statetemp2==1)//堵转标志位
	{
		Stop_flag=1;
		statetemp1=0;
	}
}
/*山外虚拟示波器*/
void vcan_sendware(uint8 *wareaddr, uint32 waresize)
{
    #define CMD_WARE   3
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令

    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //先发送前命令
    uart_putbuff(VCAN_PORT, wareaddr, waresize);    //发送数据
    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //发送后命令
}


