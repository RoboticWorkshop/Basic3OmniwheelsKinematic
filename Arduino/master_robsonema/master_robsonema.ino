#include "timer.h"
#include "i2cinterface.h"
#include "interface.h"
#include "roshandler.h"

ros::NodeHandle nh;

byte x=0, check_mode = 0;
bool enc_lock=false, play = false, check = false;

void setup() {
  pinMode(36,OUTPUT);
  pinMode(8,OUTPUT);
  digitalWrite(36,HIGH);
  digitalWrite(8,LOW);
  lcd_init(20,4);
  init_timer(100); //set timer setiap 100 ms
  init_slave_i2c(); //inisialisasi semua slave i2c
  init_ros();
  init_button(6);
  reset_encoder();
  encoder_enable();
  lcd_clear();
}

void loop(){
  //ketika timer overflow
  //================================================
  if(play==true){
    lcd_gotoxy(17,0);
    lcd_putsf("ON ");  
  }
  else{
    lcd_gotoxy(17,0);
    lcd_putsf("OFF");
  }
  /*if(check==true){
    lcd_gotoxy(17,3);
    lcd_putsf("ON ");
  }
  else{
    lcd_gotoxy(17,3);
    lcd_putsf("OFF");
  }
  lcd_gotoxy(19,1);
  lcd_puts(check_mode);*/
  if(x==0)encoder_interface();
  else if(x==1)pose_interface();
  if(!tb1){
    while(!tb1){};
    if(x==1)x=0;
    else x=1;
    digitalWrite(8,HIGH);delay(200);
    digitalWrite(8,LOW);  
    lcd_clear();
  }
  /*if(!tb2){
    while(!tb2){};
    if(++check_mode>5)check_mode=0;
  }*/
  if(!tb3){
    while(!tb3){};  
    u_x = 0;
    u_y = 0;
    u_z = 0;
    publish_pose();
    reset_encoder();
  }
  if(!tb4){
    while(!tb4){};
    imu_ref = imu_x;  
  }
  /*if(!tb5){
    if(play == false){
      while(!tb5){};
      if(check == false)check = true;  
      else check = false;        
    }  
  }*/
  if(!tb6){
    while(!tb6){};
    if(play == true)play=false;
    else play=true;
  }
  if(tim_ovf == true){ 
    //program yang dijalankan setiap 100 ms
    read_bno055();
    publish_imu();
    if(enc_lock==false){
      encoder_disable();
      read_encoder();
      publish_encoder();
      encoder_enable();
    }
    //reset timer
    tim_ovf = false;
  }
  //transmit_motor(w3,w1,w2); 
  if(play == true)transmit_motor(w3,w1,w2);
  else transmit_motor(0,0,0);
  /*else if(check == true){
    if(check_mode == 0)transmit_motor(200,200,200); 
    else if(check_mode == 1)transmit_motor(-200,-200,-200); 
    else if(check_mode == 2)transmit_motor(-200,0,200); 
    else if(check_mode == 3)transmit_motor(200,0,-200);  
    else if(check_mode == 4)transmit_motor(-200,200,-200); 
    else if(check_mode == 5)transmit_motor(200,-200,200);
  }
  else {
    transmit_motor(0,0,0); 
  }*/
  nh.spinOnce();
  delay (10);
  //================================================
}
