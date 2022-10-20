/*
 * File:   main.c
 * Author: Star Command Mechatronics Team
 *
 * Created on October 19, 2022, 6:25 PM
 */


#include "xc.h"
#pragma config FNOSC = LPRC

#define CW 1
#define CCW 0
#define REST 0
#define FASTSTEPS 38
#define SLOWSTEPS 77
#define ONEREV 400
#define ONESEC 1938

int steps = 0;

void tankTurn(int dir, int degrees);
void driveStraight();
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);

int main(void) {
    
    //SET UP STEPPER MOTORS
    
    
    
    
    while(1){
        
    }
    
    return 0;
}

void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){
    _OC1IF = 0;
    
    ++steps;
}
