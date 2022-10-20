/*
 * File:   main.c
 * Author: Star Command Mechatronics Team
 *
 * Created on October 19, 2022, 6:25 PM
 */


#include "xc.h"
#pragma config FNOSC = LPRC //31 kHz oscillator

#define CW 0
#define CCW 1
#define REST 0
#define ONEREV 200
#define ONESEC 1938
#define WHEELDIAMETER 69.5 //mm
#define TRACKWIDTH 221 //mm
#define TURNSPEED 75
#define STRAIGHTSPEED 75

//GLOBAL VARIABLES
int OC1Steps = 0;
int OC2Steps = 0;
int OC3Steps = 0;
int turnCoeff = TRACKWIDTH / (1.8 * WHEELDIAMETER);
int stepsToTake = 0;

//FUNCTION PROTOTYPES
void tankTurn(int dir, int degrees);
void driveStraight();
void setupSteppers();
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void);

int main(void) {
    
    setupSteppers();
    
    //SET UP PARAMETERS FOR STATE MACHINE
    enum {STRAIGHT1, RIGHT, STRAIGHT2, TURNAROUND} state;
    
    state = STRAIGHT1;
   
    //TIMER
    _TON = 1;       // Turn Timer1 on
    _TCKPS = 0b01;  // Chose pre-scaling as 8
    _TCS = 0;       // Internal clock source (FOSC/2)
    TMR1 = 0;       // Reset Timer1
    
    
    driveStraight();
    
    while(1){
        switch(state){
            
            case STRAIGHT1:
                
                if(TMR1 >= 2*ONESEC){
                    state = RIGHT;
                    tankTurn(90,CW);
                }
                break;
                
            case RIGHT:
                
                if(OC2Steps >= stepsToTake){
                    state = STRAIGHT2;
                    TMR1 = 0;
                    driveStraight();
                    _OC2IE = 0;
                }
                break;
                
            case STRAIGHT2:
                
                if(TMR1 >= 4*ONESEC){
                    state = TURNAROUND;
                    tankTurn(180,CCW);
                }                
                break;
                
            case TURNAROUND:
                
                if(OC2Steps >= stepsToTake){
                    state = STRAIGHT1;
                    TMR1 = 0;
                    driveStraight();
                    _OC2IE = 0;
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

    //TURN ON INTERRUPT
    _OC2IE = 1;
    
    OC2Steps = 0;
    
    //SET PERIOD AND DUTY CYCLE
    OC2RS = TURNSPEED;
    OC2R = TURNSPEED/2;
    OC3RS = TURNSPEED;
    OC3R = TURNSPEED/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = dir;
    _LATA1 = dir;
}
void setupSteppers(){
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
    
    //SET UP DIRECTION PINS
    _TRISA0 = 0;
    _ANSA0 = 0;
    _ANSA1 = 0;
    _TRISA1 = 0;
    
}
void driveStraight(){
    
    //SET PERIOD AND DUTY CYCLE
    OC2RS = STRAIGHTSPEED;
    OC2R = STRAIGHTSPEED/2;
    OC3RS = STRAIGHTSPEED;
    OC3R = STRAIGHTSPEED/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1;
    _LATA1 = 0;
    
}