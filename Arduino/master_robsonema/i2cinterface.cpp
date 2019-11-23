#ifndef i2cinterface_cpp
#define i2cinterface_cpp

#include <Arduino.h>
#include "i2cinterface.h"
#include "interface.h"
#include "roshandler.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55); //set alamat slave BNO055

volatile float imu_x = 0, imu_y = 0, imu_z = 0, imu_ref=0; // variael sensor kompas
volatile int enc[2]={0,0}; // variabel encoder
byte slave_addr[6], en = 12;

void init_slave_i2c(){
  Wire.begin();
  pinMode(en,OUTPUT);
  set_slave('L', KIRI);
  set_slave('M', TENGAH);
  set_slave('R', KANAN);
  set_slave('D', DRIBBLER);
  set_slave('Q', EXT_ENC_1);
  set_slave('W', EXT_ENC_2);
  init_bno055(); //deklarasi sensor kompas bno055
  encoder_disable();
}

void init_bno055(){
  if(!bno.begin()){
    lcd_gotoxy(5,1);
    lcd_putsf("WARNING!!!");
    lcd_gotoxy(0,2);
    lcd_putsf("No BNO055 Connected!");
    delay(2000);
  }
  else {
    lcd_gotoxy(0,1);
    lcd_putsf("Compass Detected");
    lcd_gotoxy(0,2);
    lcd_putsf("BNO055 Connected...");
    delay(500);
  } 
  bno.setExtCrystalUse(true); 
}

void read_bno055(){
  sensors_event_t event;
  bno.getEvent(&event);
  imu_x = event.orientation.x; // arah hadap robot
  imu_y = event.orientation.y;
  imu_z = event.orientation.z;
}

void set_slave(char part, unsigned char addr){
  switch(part){
    case 'L': slave_addr[0] = addr;break;//kontroler motor kiri
    case 'M': slave_addr[1] = addr;break;//kontroler motor tengah
    case 'R': slave_addr[2] = addr;break;//kontroler motor kanan
    case 'D': slave_addr[3] = addr;break;//kontroler motor dribbler 
    case 'Q': slave_addr[4] = addr;break;//kontroler encoder external 1
    case 'W': slave_addr[5] = addr;break;//kontroler encoder external 2
  }
}

void transmit_motor(int lpwm, int mpwm, int rpwm){
  byte dir[3]={0,0,0};
  if(lpwm>0)dir[0]=1;//cw
  else if(lpwm<0)dir[0]=2;//ccw
  else dir[0]=0;//stop
  if(mpwm>0)dir[1]=1;//cw
  else if(mpwm<0)dir[1]=2;//ccw
  else dir[1]=0;//stop
  if(rpwm>0)dir[2]=1;//cw
  else if(rpwm<0)dir[2]=2;//ccw
  else dir[2]=0;//stop
  //transmit data ke kontroler motor kiri
  Wire.beginTransmission(slave_addr[0]);//alamat slave kontroler motor kiri
  Wire.write((byte)(1));//kode pengiriman arah dan kecepatan motor
  Wire.write((byte)(dir[0]));//data arah putar motor
  Wire.write((byte)(abs(lpwm)>>8));
  Wire.write((byte)(abs(lpwm)));
  Wire.endTransmission();
  //transmit data ke kontroler motor tengah
  Wire.beginTransmission(slave_addr[1]);//alamat slave kontroler motor tengah
  Wire.write((byte)(1));//kode pengiriman arah dan kecepatan motor
  Wire.write((byte)(dir[1]));//data arah putar motor
  Wire.write((byte)(abs(mpwm)>>8));
  Wire.write((byte)(abs(mpwm)));
  Wire.endTransmission();
  //transmit data ke kontroler motor kanan
  Wire.beginTransmission(slave_addr[2]);//alamat slave kontroler motor kanan
  Wire.write((byte)(1));//kode pengiriman arah dan kecepatan motor
  Wire.write((byte)(dir[2]));//data arah putar motor
  Wire.write((byte)(abs(rpwm)>>8));
  Wire.write((byte)(abs(rpwm)));
  Wire.endTransmission();
}

void transmit_dribbler(int lpwm, int rpwm){
  byte dir[2]={0,0};
  if(lpwm>0)dir[0]=1;//cw
  else if(lpwm<0)dir[0]=2;//ccw
  else dir[0]=0;//stop
  if(rpwm>0)dir[1]=1;//cw
  else if(rpwm<0)dir[1]=2;//ccw
  else dir[1]=0;//stop
  Wire.beginTransmission(slave_addr[3]);//alamat slave kontroler dribbler
  Wire.write((byte)(dir[0]));//data arah putar motor
  Wire.write((byte)(dir[1]));//data arah putar motor
  Wire.write((byte)(abs(lpwm)));
  Wire.write((byte)(abs(rpwm)));
  Wire.endTransmission();
}

void read_encoder(){
  Wire.requestFrom(slave_addr[4],2);//enc1
  enc[0] =0;
  enc[0] |= Wire.read();
  enc[0] = enc[0] << 8;
  enc[0] |= Wire.read();
  Wire.requestFrom(slave_addr[5],2);//enc2
  enc[1] =0;
  enc[1] |= Wire.read();
  enc[1] = enc[1] << 8;
  enc[1] |= Wire.read();
}

void reset_encoder(){
  //kiri
  Wire.beginTransmission(slave_addr[0]);  
  Wire.write((byte)(2));//kode pengiriman reset encoder
  Wire.endTransmission();
  //tengah
  Wire.beginTransmission(slave_addr[1]);  
  Wire.write((byte)(2));//kode pengiriman reset encoder
  Wire.endTransmission();
  //kanan
  Wire.beginTransmission(slave_addr[2]);  
  Wire.write((byte)(2));//kode pengiriman reset encoder
  Wire.endTransmission();
}

void encoder_enable(){
  digitalWrite(en,HIGH);
}

void encoder_disable(){
  digitalWrite(en,LOW);
}

#endif
