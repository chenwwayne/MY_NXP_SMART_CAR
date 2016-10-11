#ifndef _KEY_OLED_H
#define _KEY_OLED_H

void Swithch_Scan(void);
void OLED_Display(void);
void Matrix_KeyScan(void);
uint8 Matrix_Key();

//#define Switch2 (gpio_get(PTD1))
//#define Switch3 (gpio_get(PTC17))
//#define Switch4 (gpio_get(PTC13)) 

#endif