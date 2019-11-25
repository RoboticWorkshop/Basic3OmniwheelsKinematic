#ifndef INTERFACE_H
#define INTERFACE_H

extern volatile byte pintombol[6];

#define tb1 digitalRead(pintombol[0])==HIGH
#define tb2 digitalRead(pintombol[1])==HIGH
#define tb3 digitalRead(pintombol[2])==HIGH
#define tb4 digitalRead(pintombol[3])==HIGH
#define tb5 digitalRead(pintombol[4])==HIGH
#define tb6 digitalRead(pintombol[5])==HIGH

void lcd_init(byte coloumn, byte row);
void init_button(byte num);
void lcd_gotoxy(byte x, byte y);
void lcd_putsf(String words);
void lcd_puts(int num);
void lcd_clear();
void encoder_interface();
void pose_interface();


#endif
