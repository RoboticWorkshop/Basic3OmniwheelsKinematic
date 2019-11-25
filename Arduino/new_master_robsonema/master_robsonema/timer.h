#ifndef TIMER_H
#define TIMER_H

extern volatile bool tim_ovf;

void init_timer(int milsec);

#endif
