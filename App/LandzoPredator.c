#include "common.h"
#include "include.h"
#include "LandzoPredator.h"


uint8_t  LandzoRAM[ ] ={//a0
  0x03,0x00,0x0c,0x00,0X11,0X00,0X12,0X01,0X13,0X17,0X15,0X02,0X16,0X03,0X03,0X00,
  0X17,0X00,0X18,0Xa0,0X19,0X00,0X1a,0X78,0X1b,0X2a,0X1f,0X20,0X20,0X02,0X09,0X03,
  0X21,0X00,0X22,0X20,0X23,0X33,0X24,0X8a,0X25,0X7a,0X2A,0X00,0X2B,0X00,0X2c,0X00,
  0X2d,0X4f,0X2e,0X96,0X2f,0X00,0X3a,0X00,0X3b,0X60,0X3e,0X34,0X3f,0Xb0,0X40,0X4c,
  0X41,0X3b,0X42,0X30,0X43,0X25,0X44,0X1d,0X45,0X15,0X46,0X10,0X47,0X0f,0X48,0X0d,
  0X49,0X0b,0X4b,0X09,0X4c,0X09,0X4e,0X08,0X4f,0X06,0X50,0X05,0X51,0X2f,0X52,0X90,
  0X53,0X01,0X54,0X87,0X56,0X70,0X57,0X32,0X58,0X8b,0X59,0X06,0X5a,0X9b,0X5b,0X35,
  0X64,0X02,0X67,0X80,0X68,0X80,0X69,0XA0,0X6a,0X89,0X70,0Xae,0X71,0Xa6,0X72,0X4f,
  0X73,0X5f,0X74,0X27,0X80,0X41,0X81,0X08,0X82,0X16,0X83,0X24,0X84,0X24,0X85,0X2a,
  0X86,0X4f,0X89,0Xb2,0X8a,0Xaa,0X8b,0X02,0X8e,0X02,0X8f,0X70,0X95,0X14,0X9d,0X36,
  0X9e,0X04,0Xa1,0X27,0Xa2,0X12,0Xa3,0X2d,0Xa4,0X08,0Xa5,0X2e,0Xa6,0X04,0Xa7,0X80,
  0Xa8,0X80,0Xa9,0X21,0Xaa,0X21,0Xab,0X21,0Xac,0X0a,0Xad,0Xf0,0Xae,0Xff,0Xb1,0Xb0,
  0Xb2,0Xb0,0Xb6,0X80,0Xb7,0X80,0Xb8,0X80,0Xb9,0X00,0Xe7,0X7c,0Xf0,0X00,0Xf1,0X01,
  0Xf1,0X01,0Xf1,0X01,
};




/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：BFDly_ms
*  功能说明：延时函数
*  参数说明：ms
*  函数返回：无
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/

void BFDly_ms(uint8_t ms)
{
  volatile uint16_t ii,jj,n;
  if (ms<1) ms=1;
  for(ii=0;ii<ms;ii++)
    //  for(jj=0;jj<1335;jj++);     //16MHz--1ms
    //for(jj=0;jj<4005;jj++);    //48MHz--1ms  
    //for(jj=0;jj<5341;jj++);    //64MHz--1ms  
    for(jj=0;jj<18200;jj++);     //200MHz--1ms  
} 

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：BFdelay_1us
*  功能说明：延时函数
*  参数说明：us
*  函数返回：无
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/
void BFdelay_1us(uint8_t us)                 //1us延时函数
{
  volatile uint8_t i ,j ;
  if(us < 1 )  us = 1 ;
  for(i=0;i<us;i++) 
  {
    for(j = 0 ;j < 15 ;j ++);
    
  }
  
}

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：iic_start
*  功能说明： 启动I2C总线子程序
*  参数说明：无
*  函数返回：无
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/
void iic_start(void)
{
  BFSDA = 1;  
  BFdelay_1us(1);      // 延时1us 
  BFCLK = 1;
  BFdelay_1us(1);      // 延时5us 
  BFSDA = 0;
  BFdelay_1us(1);  
  BFCLK = 0;
  BFdelay_1us(2);
}

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：iic_stop
*  功能说明：停止I2C总线数据传送子程序
*  参数说明：无
*  函数返回：无
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/
void iic_stop(void)
{ 
  BFSDA = 0;   	   //时钟保持高，数据线从低到高一次跳变，I2C通信停止
  BFdelay_1us(1);      // 延时1us 
  BFCLK = 1;
  BFdelay_1us(1);
  BFSDA = 1;
  BFdelay_1us(1);
  BFCLK = 0;
  BFdelay_1us(2);
}

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：slave_ACK
*  功能说明：从机发送应答位子程序
*  参数说明：无
*  函数返回：无
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/
void slave_ACK(void)
{
  BFSDA = 0; 
  BFdelay_1us(1);      // 延时1us 
  BFCLK = 1;
  BFdelay_1us(1);			
  BFSDA = 1;
  BFdelay_1us(1);      // 延时1us 
  BFCLK = 0;
  BFdelay_1us(2);
}

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：slave_NOACK
*  功能说明：从机发送非应答位子程序，迫使数据传输过程结束
*  参数说明：无
*  函数返回：无
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/
void slave_NOACK(void)
{ 
  BFSDA = 1;  
  BFdelay_1us(1);      // 延时1us 
  BFCLK = 1;
  BFdelay_1us(3);
  BFSDA = 0;
  BFdelay_1us(1);      // 延时1us 
  BFCLK = 0;
}

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：check_ACK
*  功能说明： 主机应答位检查子程序，迫使数据传输过程结束
*  参数说明：无
*  函数返回：无
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/
uint8_t check_ACK(void)
{ 
  uint8_t check ;
  
  BFSDA = 1; 
  
  //gpio_init (PTC11, GPI,1); 
  //port_init_NoALT (PTC11,PF | PULLUP );
  
  BFCLK = 0;
  BFDDRA = 0 ;
  BFSDAI = 1 ;
  
  BFdelay_1us(1);      // 延时1us 
  BFCLK = 1;
  check = 0;
  if(BFSDAI == 1)    // 若BFSDA=1 表明非应答，置位非应答标志F0
    check = 1;
  BFdelay_1us(1);      // 延时1us 
  BFCLK = 0;
  
  //gpio_init (PTC11, GPO,1);      
  
  BFDDRA = 1 ;
  BFSDA = 1 ;
  return  check ;
}

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：IICSendByte
*  功能说明：发送一个字节
*  参数说明：ch
*  函数返回：无
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/
void IICSendByte(uint8_t ch)
{ 
  uint8_t n=8;     // 向BFSDA上发送一位数据字节，共八位
  while(n--)
  { 
    if((ch&0x80) == 0x80)    // 若要发送的数据最高位为1则发送位1
    {
      BFSDA = 1;    // 传送位1
      BFdelay_1us(1);
      BFCLK = 1;
      BFdelay_1us(2);
      BFCLK = 0;  
      BFdelay_1us(1);
      BFSDA = 0;
      BFdelay_1us(1);
      
    }
    else
    {  
      BFSDA = 0;    // 否则传送位0
      BFdelay_1us(1);
      BFCLK = 1;
      BFdelay_1us(2);
      BFCLK = 0;  
      BFdelay_1us(2);
    }
    ch = ch<<1;    // 数据左移一位
  }
}

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：IICreceiveByte
*  功能说明：接收一字节子程序
*  参数说明：无
*  函数返回：
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/
uint8_t IICreceiveByte(void)
{
  uint8_t n=8;    // 从BFSDA线上读取一上数据字节，共八位
  uint8_t tdata = 0;
  while(n--)
  {
    
    BFSDAI = 1;
    BFdelay_1us(1);
    BFCLK=0;
    BFdelay_1us(2);
    BFCLK = 1;
    BFdelay_1us(2);
    tdata = tdata<<1;    // 左移一位，或_crol_(temp,1)
    if(BFSDAI == 1)
      tdata = tdata|0x01;    // 若接收到的位为1，则数据的最后一位置1
    else 
      tdata = tdata&0xfe;    // 否则数据的最后一位置0
    BFCLK=0;
  }
  return(tdata);
}

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：writeNbyte
*  功能说明：发送n位数据子程序
*  参数说明：slave_add从机地址,n要发送的数据个数
*  函数返回：无
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/
uint8_t writeNbyte(uint8_t slave_add, uint8_t *slave_data,uint16_t n)
{          
  uint8_t send_da,check = 1;
  uint16_t i=0;
  uint8_t *Send_databuff ;
  Send_databuff = slave_data ;
  //  uart_putchar(UART0,slave_data[0]);
  //   uart_putchar(UART0,n>>8);
  iic_start();                // 启动I2C
  IICSendByte(slave_add);     // 发送地址位
  check = check_ACK();                // 检查应答位
  //    uart_putchar(UART0,check);
  if(check == 1)
  { 
    return IICEorr;    // 若非应答表明器件错误或已坏，置错误标志位SystemError
  }
  while(n--)
  { 
    send_da = Send_databuff[i++];	
    IICSendByte(send_da);     
    check= check_ACK();    // 检查应答位
    //uart_putchar(UART0,send_da);
    if (check == 1)
    {
      return IICEorr;    // 若非应答表明器件错误或已坏，置错误标志位SystemError
    }
  }
  iic_stop();         // 全部发完则停止
  
  return IICOK;
}

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：receiveNbyte
*  功能说明：接收n位数据子程序
*  参数说明：slave_add从机地址,要接收的数据个数
*  函数返回：无
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/
uint8_t receiveNbyte(uint8_t slave_add,uint8_t *rece_data, uint16_t n)
{ 
  uint8_t receive_da,check;
  uint16_t i=1;
  uint8_t *rece_data_buff ;
  rece_data_buff = rece_data ;
  
  iic_start();
  IICSendByte(0XDC);
  BFdelay_1us(1);      // 延时1us 
  check =check_ACK();
  if(check == 1)
  {
    return IICEorr ;
  }
  IICSendByte(rece_data_buff[0]);
  BFdelay_1us(1);      // 延时1us 
  check =check_ACK();
  if(check == 1)
  {
    return IICEorr ;
  }
  // uart_putchar(UART0,rece_data_buff[0]);
  iic_start();
  IICSendByte(slave_add);
  BFdelay_1us(1);      // 延时1us 
  check =check_ACK();
  if(check == 1)
  {
    return IICEorr ;
  }
  
  for(;n > 1;n--)
  { 
    receive_da=IICreceiveByte();
    rece_data_buff[i++]=receive_da;
    slave_ACK();    // 收到一个字节后发送一个应答位
    
    //       uart_putchar(UART0,receive_da);
    
  }
  
  receive_da=IICreceiveByte();
  rece_data_buff[i++]=receive_da;
  slave_NOACK();    // 收到最后一个字节后发送一个非应答位
  //   uart_putchar(UART0,receive_da);
  iic_stop();
  return IICOK;
  
}

/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：IICWriteGpio_inint
*  功能说明：初始化程序
*  参数说明：无
*  函数返回：无
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/
void IICWriteGpio_inint(void)
{
  gpio_init (PTE0, GPO,HIGH); 
  gpio_init (PTE1, GPO,HIGH);
  gpio_init (PTE31 , GPO,HIGH);
}
/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：LandzoIICEEROM_INIT
*  功能说明：初始化程序
*  参数说明：无
*  函数返回：无
*  修改时间：2014-9-13
*  备    注：
*************************************************************************/
uint8_t LandzoIICEEROM_INIT(void){
  uint8_t LandzoOK ;
  uint16_t i ,EROMCont = 212 ;
  uint8_t IIClandzo_buff[514] ;
  

  IICWriteGpio_inint() ;    //初始化读入IIC端口
  for(i = 0 ; i < EROMCont ;)
  {
    LandzoOK = writeNbyte(0XDC,&LandzoRAM[i],2);
    if(!LandzoOK) break ;
    i = i +2 ;
  }
 
  BFDly_ms(100) ;                 //延迟100us
  
  if( !LandzoOK )
  {
    IICWriteGpio_inint() ;    //初始化读入IIC端口
    for(i = 0 ; i < EROMCont ;)
    {
      LandzoOK = writeNbyte(0XDC,&LandzoRAM[i],2);
      if(!LandzoOK) break ;
      i = i +2 ;
    }
  }
  
  if(!LandzoOK)
    {return IICEorr;}
  
  for(i = 0 ; i < EROMCont ; )
  {
    IIClandzo_buff[i] = LandzoRAM[i] ; 
  //  uart_putchar(UART0,IIClandzo_buff[i]);
    BFdelay_1us(10);      // 延时1us 
    LandzoOK = receiveNbyte(0XDD,&IIClandzo_buff[i],1) ;
    i += 2;
  }
  
  for(i = 0 ;i < EROMCont ; i++)
  {
    if(LandzoRAM[i] != IIClandzo_buff[i] )
    return IICEorr ;
  }
   
 //  uart_putchar(UART0,IICOK);
  return  IICOK ;

}

/*********************************************************** 
函数名称：Predator_INC
函数功能：摄像头初始化
入口参数：C6 为场中断， C4为行中断
DMA4通道采样通道，选择DMA_Always_EN1为通道
出口参数：无 
备 注： 
***********************************************************/
extern uint8_t  BUFF[500] ;
extern uint8_t  ADdata[DATALINE][DATACOUNT];
extern uint8_t  LinADCout    ;

void LandzoCamera_init (void)
{
      gpio_init(PTE16, GPI, 1);
      gpio_init(PTE17, GPI, 1);
      gpio_init(PTE18, GPI, 1);
      gpio_init(PTE19, GPI, 1);
      gpio_init(PTE20, GPI, 1);
      gpio_init(PTE21, GPI, 1);
      gpio_init(PTE22, GPI, 1);
      gpio_init(PTE23, GPI, 1);
      gpio_init (PTE31, GPI,1);
      port_init (PTA1,  IRQ_FALLING| PF | ALT1 | PULLUP );         //场中断
		 
      port_init (PTA6,  IRQ_FALLING| PF | ALT1 | PULLUP );       //行中断 
      
      
      port_init_NoALT(PTE31,DMA_FALLING );
      dma_portx2buff_init (DMA_CH2, (void *)&PTE_B2_IN,ADdata,PTE31,DMA_BYTE1, DATACOUNT); //初始化DMA模块   
        
} 


/*************************************************************************
*                           蓝宙电子工作室
*
*  函数名称：CCDOledPrint
*  功能说明：液晶屏显示程序
*  参数说明：ADdataCount  每行的采样点数 ；LineCount 一场采样行数 ；
PredatorFAV  二值化阈值
*  函数返回：无
*  修改时间：2014-9-5
*  备    注：
*************************************************************************/
void PredatorOledPrint(uint16_t ADdataCount,uint16_t LineCount,uint16_t PredatorFAV,
                       uint8 *dataAD )
{
  uint16_t PCount ,OLECount ;
  uint8_t plineCount ;
  //    OLED_Print_Str(5,0 , "Landzo CCD Dis");
  
  //     gpio_Interrupt_init(PORTD,14, GPI_DOWN, GPI_DISAB) ;          //场中断
  //     gpio_Interrupt_init(PORTD,13, GPI_DOWN, GPI_DISAB) ;          //行中断 
  if(LineCount > 8) LineCount = 8 ;
  for(plineCount = 0 ;plineCount < LineCount;plineCount ++)
  {
    OLED_Print_Str(0,plineCount , "               ");
    for(PCount = 0;PCount<ADdataCount ;PCount++)
    {
      OLECount = PCount/2 ;
      if(OLECount < 127){
        OLED_Set_Pos(OLECount,plineCount);
        if(dataAD[PCount]>PredatorFAV)
        {
          OLED_WrDat(0x0);
        }
        else
          OLED_WrDat(0xf0);
      }
    }
  }
  //    gpio_Interrupt_init(PORTD,14, GPI_DOWN, RING) ;
}
