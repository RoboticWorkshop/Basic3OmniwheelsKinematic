//new slave driver

#include <Wire.h>
#include "driver.h"
#include "gerakan.h"
#define KIRI    11
#define TENGAH  12
#define KANAN   13

byte stat=0;

void receiveEvent(){
  stat=((byte)Wire.read());
  if(stat==1){
    mov = ((byte)Wire.read());
    pwmVal = 0;
    pwmVal |= ((int)(Wire.read() << 8));
    pwmVal |= ((int)(Wire.read()));
  }
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  //Wire.begin(KIRI);
  //Wire.begin(TENGAH);
  Wire.begin(KANAN);
  Wire.onReceive(receiveEvent);
  setup_pwm16();
}

void loop() {
  // put your main code here, to run repeatedly:
  run_program();
}
