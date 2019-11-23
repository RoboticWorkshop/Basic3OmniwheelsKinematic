#ifndef timer_cpp
#define timer_cpp

#include <Arduino.h>
#include "timer.h"
#define prescaler    256
#define f_cpu        16000000

int start = 0;
volatile bool tim_ovf = false;

ISR(TIMER1_OVF_vect){
  tim_ovf = true;
  TCNT1 = start;
}

void init_timer(int milsec){
  start = 65535 - (milsec*(f_cpu/1000)/prescaler); //hitung rumus timer1
  TCCR1A = 0; //set register timer1
  TCCR1B = (1 << CS12) | (0 << CS11) | (0 << CS10);//set register timer1
  TCNT1 = start; //set default start hitungan timer
  TIMSK1 |= (1<<TOIE1); //register interupsi dibaca
  sei();
  tim_ovf = false;
}

#endif
