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
#define WHEELDIAMETER 69.5 //mm
#define TRACKWIDTH 221 //mm

//GLOBAL VARIABLES
int OC1Steps = 0;
int OC2Steps = 0;
int OC3Steps = 0;
float turnCoeff = TRACKWIDTH / (1.8 * WHEELDIAMETER);
int stepsToTake = 0;
int turnDir = 0;

//FUNCTION PROTOTYPES
void tankTurn(int dir, int degrees);
void driveStraight();
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void);

int main(void) {
    
    //SET UP STEPPER MOTORS
    //RIGHT
    OC2CON1 = 0;
    OC2CON2 = 0;
    OC2CON1bits.OCTSEL = 0b111;
    OC2CON2bits.SYNCSEL = 0x1F;
    OC2CON2bits.OCTRIG = 0;
    OC2CON1bits.OCM = 0b110;
    
    _OC2IP = 4; // Select OCx interrupt priority
    _OC2IE = 1; // Enable OCx interrupt
    _OC2IF = 0; // Clear OCx interrupt flag
    
    //LEFT
    OC3CON1 = 0;
    OC3CON2 = 0;
    OC3CON1bits.OCTSEL = 0b111;
    OC3CON2bits.SYNCSEL = 0x1F;
    OC3CON2bits.OCTRIG = 0;
    OC3CON1bits.OCM = 0b110;
    
    _OC3IP = 4; // Select OCx interrupt priority
    _OC3IE = 1; // Enable OCx interrupt
    _OC3IF = 0; // Clear OCx interrupt flag
    
    
    //SET UP PARAMETERS FOR STATE MACHINE
    enum {RIGHT, TURNAROUND, STRAIGHT} state;
    enum {RIGHT, TURNAROUND} prevState;
    
    state = STRAIGHT;
    prevState = TURNAROUND;
   
    //TIMER
    _TON = 1;       // Turn Timer1 on
    _TCKPS = 0b01;  // Chose prescaling
    _TCS = 0;       // Internal clock source (FOSC/2)
    TMR1 = 0;       // Reset Timer1
    
    while(1){
        switch(state){
            
            case STRAIGHT:
                
                
                if(TMR1 >= 2*ONESEC){
                    if(prevState == RIGHT){
                        state = TURNAROUND;
                    }else if(prevState == TURNAROUND){
                        state = RIGHT;
                    }
                }
                break;
                
            case RIGHT:
                
                
                
                if(OC2Steps >= stepsToTake){
                    state = STRAIGHT;
                    prevState = RIGHT;
                }
                break;
                
            case TURNAROUND:
                
                
                if(OC2Steps >= stepsToTake){
                    state = STRAIGHT;
                    prevState = TURNAROUND;
                }
                break;
        }
    }
    
    return 0;
}

void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void){
    _OC1IF = 0;
    
    ++OC1Steps;
}
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void){
    _OC2IF = 0;
    
    ++OC2Steps;
}
void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void){
    _OC3IF = 0;
    
    ++OC3Steps;
}
void tankTurn(int dir, int degrees){
    stepsToTake = turnCoeff * degrees;
    
    turnDir = dir;
}
