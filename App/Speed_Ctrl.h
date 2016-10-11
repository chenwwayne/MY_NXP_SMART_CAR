#ifndef __SPEED_CTRL_
#define __SPEED_CTRL_ 
#include  "include.h"
/*速度PID结构体*/
struct pid
{
	float SetSpeed;
	float ActualSpeed;
	float MotorDuty;
	float err[3];
	float Kp,Ki,Kd;
};
void PID_Init(void);
float PID_Realize(struct pid *p);
void SpeedCtrl(void);
void Motor_Stall(void);
void PID_Fix(void);
void vcan_sendware(uint8 *wareaddr, uint32 waresize);

#endif 

/*  电机  恢复时间
	 p7     45 
	 p12   24
	 p  17   27
	 p22     20 
	 p27     16
	 p32     15
	 p37     13   
	 p42     12.5
*/



