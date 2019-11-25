#ifndef interface_cpp
#define interface_cpp

#include "interface.cpp"
#include "i2cinterface.h"
#include "roshandler.h"
#include <Arduino.h>
#include <LiquidCrystal.h>
#define RS  24
#define E   26
#define D4  28
#define D5  30
#define D6  32
#define D7  34

LiquidCrystal lcd(RS,E,D4,D5,D6,D7);

volatile byte pintombol[6]={2,3,4,5,6,7};
char buff[33];

void lcd_init(byte coloumn, byte row){
  lcd.begin(coloumn, row);  
}

void init_button(byte num){
  byte i = 0;
  for(i = 0;i<num;i++){
    pinMode(pintombol[i],INPUT_PULLUP);
  }
}

void lcd_gotoxy(byte x, byte y){
  lcd.setCursor(x,y);  
}

void lcd_putsf(String words){
  lcd.print(words);
}

void lcd_clear(){
  lcd.clear();  
}

void lcd_puts(int num){
  lcd.print(num);  
}

void encoder_interface(){
  lcd_gotoxy(0,0);
  lcd_putsf("<Encoder>");
  lcd_gotoxy(1,1);
  sprintf(buff,"enc1 = %06d ", enc[0]);
  lcd.print(buff);
  lcd_gotoxy(1,2);
  sprintf(buff,"enc2 = %06d ", enc[1]);
  lcd.print(buff);
}

void pose_interface(){
  lcd_gotoxy(0,0);
  lcd_putsf("<Pose>");
  lcd_gotoxy(1,1);
  lcd.print("x =");lcd.print(pose_x);lcd.print(" ");lcd.print("r:");lcd.print(imu_ref);lcd.print(" ");
  lcd_gotoxy(1,2);
  lcd.print("y =");lcd.print(pose_y);lcd.print(" ");lcd.print("c:");lcd.print(imu_x);lcd.print(" ");
  lcd_gotoxy(1,3);
  lcd.print("th=");lcd.print(pose_z);lcd.print("   ");
}

#endif
