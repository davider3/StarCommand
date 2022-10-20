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
#define ONEREV 200
#define ONESEC 1938

int steps = 0;

void tankTurn(int dir, int degrees);
void driveStraight();
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);

int main(void) {
    
    //SET UP STEPPER MOTORS
    
    //RIGHT
    OC2CON1 = 0;
    OC2CON2 = 0;
    OC2CON1bits.OCTSEL = 0b111;
    OC2CON2bits.SYNCSEL = 0x1F;
    OC2CON2bits.OCTRIG = 0;
    OC2CON1bits.OCM = 0b110;
    
    //LEFT
    OC3CON1 = 0;
    OC3CON2 = 0;
    OC3CON1bits.OCTSEL = 0b111;
    OC3CON2bits.SYNCSEL = 0x1F;
    OC3CON2bits.OCTRIG = 0;
    OC3CON1bits.OCM = 0b110;
    
    
    //SET UP PARAMETERS FOR STATE MACHINE
    enum {STRAIGHT, RIGHT, TURNAROUND} state;
    
    while(1){
        switch(state){
            
            case STRAIGHT:
                
                
                
                break;
                
            case RIGHT:
                
                break;
                
            case TURNAROUND:
                
                break;
        }
    }
    
    return 0;
}

void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){
    _OC1IF = 0;
    
    ++steps;
}
