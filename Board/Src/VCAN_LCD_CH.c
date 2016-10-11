#include "common.h"


#include "VCAN_lcd.h"



void LCD_FChar_CH (Site_t site,const uint8 *str,uint16 Color,uint16 bkColor)
{
    uint8 i,j;
    //uint8 *pbuf = str;
    uint8 tmp_char=0;

    const Size_t size   = {LCD_CH_W,LCD_CH_H};

    LCD_PTLON(site, size);              //开窗
    LCD_RAMWR();                        //写内存

    for (i=0;i < LCD_CH_SIZE;i++)
    {
        tmp_char = *str++;

        for(j=0;j<8;j++)
        {
            if(tmp_char & (0x80))
            {
                LCD_WR_DATA( Color );
            }
            else
            {
                LCD_WR_DATA( bkColor );
            }
            tmp_char <<= 1;
        }
    }
}

void LCD_FStr_CH (Site_t site,const uint8 *str,uint16 num,uint16 Color,uint16 bkColor)
{
    while(num--)
    {
        if(site.x>(LCD_W-16))
        {
             /*换行*/
            site.x =0;
            site.y +=LCD_CH_H;
        }
        if(site.y >(LCD_H-LCD_CH_W))
        {
             /*重新归零*/
             site.y =0;
             site.x =0;
        }
        LCD_FChar_CH(site,str,Color,bkColor);
        str += LCD_CH_SIZE ;
        site.x += LCD_CH_W ;
    }
}

