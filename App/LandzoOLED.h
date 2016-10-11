#ifndef _LANDZO_OELD_H
#define _LANDZO_OELD_H

#include "common.h"
#include "include.h"
/********************************************************************/
/*-----------------------------------------------------------------------
LCD_init          : OLED初始化

编写日期          ：2012-11-01
最后修改日期      ：2012-11-01
-----------------------------------------------------------------------*/
 extern uint8_t lanzhou96x64[768];
 void OLED_Init(void);
 void OLED_CLS(void);
 void OLED_P6x8Str(uint8_t x,uint8_t y,uint8_t ch[]);
 void OLED_P8x16Str(uint8_t x,uint8_t y,uint8_t ch[]);
 void OLED_P14x16Str(uint8_t x,uint8_t y,uint8_t ch[]);
 void OLED_HEXACSII(uint16_t hex,uint8_t* Print);
 void OLED_Print_Num(uint8_t x, uint8_t y, uint16_t num);
 void OLED_Print_Str(uint8_t x, uint8_t y, uint8_t ch[]);
 void OLED_PutPixel(uint8_t x,uint8_t y);
 void OLED_Rectangle(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t gif);
 void OLED_Set_Pos(uint8_t x, uint8_t y);
 void OLED_WrDat(uint8_t data);
 void Draw_LibLogo(void);
 void Draw_Landzo(void);
 void Draw_BMP(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint8_t bmp[]);
 void OLED_Fill(uint8_t dat);
 void Dly_ms(uint16_t ms);
 void OLED_Wr6Dat(uint8_t data);
 
 void Dis_Float(unsigned char Y,unsigned char X,double real,unsigned char N) ;
 void Dis_Float2(unsigned char Y,unsigned char X,double real,unsigned char N1,unsigned char N2) ;
 void Dis_SNum(uint8 x, uint8 y, uint16 num,uint8 N);
 void Dis_Num(uint8 x, uint8 y, uint16 num,uint8 N);
 void OLED_IMG(uint8 (*tempimg)[128],uint16 center);



/********************************************************************/

#endif
