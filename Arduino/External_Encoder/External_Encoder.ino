#include<Encoder.h>
#include<Wire.h>
#define chA   2
#define chB   3
#define en    4
#define EXT_ENC_1   16
#define EXT_ENC_2   17

Encoder enc(chA, chB);

long newPos = 0, oldPos = 999, enc_data;
byte addr = 11, stat = 0;
bool upd = true;
int val = 0, val1 = 800;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Wire.begin(EXT_ENC_1);
  Wire.begin(EXT_ENC_2);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  pinMode(en, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  newPos = enc.read()/4;
  if(newPos != oldPos){
    oldPos = newPos;
    Serial.println(newPos);  
  }
  if(digitalRead(en)==HIGH)enc_data = newPos;
}

void receiveEvent(){
  stat=((byte)Wire.read());
  if(stat == 2){
    newPos = 0; oldPos = 0;
    enc.write(0);
    enc_data = 0;  
  }  
}

void requestEvent(){
  Wire.write((byte)(enc_data >> 8));
  Wire.write((byte)(enc_data));
}
