#ifndef gerakan_cpp
#define gerakan_cpp

#include <Arduino.h>
#include "gerakan.h"
#include "driver.h"
//11 = kiri, 12 = belakang, 13 = kanan

void run_program(){
  switch(mov){
    case 0: dir=0; break;
    case 1: dir=1; break;
    case 2: dir=2; break;
  }  
  motor(dir, pwmVal);
}

#endif
