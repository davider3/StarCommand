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
#define TURNSPEED 50
#define FAST 50 //TODO:TRIAL AND ERROR TO DECIDE THE BEST VALUE FOR SPEED
#define SLOW 150
#define SORTAFAST 120


//GLOBAL VARIABLES
int OC1Steps = 0;
int OC2Steps = 0;
int OC3Steps = 0;
float turnCoeff = TRACKWIDTH / (1.8 * WHEELDIAMETER); //1.7666
int stepsToTake = 0;


//FUNCTION PROTOTYPES
//SETUP FUNCTIONS
void setupSteppers();
void setupQRDs();
void setupTimer();

//CONTROL FUNCTIONS
void tankTurn(int degrees, int dir);
void driveStraight();
void slightRight();
void slightLeft();
void hardRight();
void hardLeft();
void search();

//CHECK STATE FUNCTIONS
int rightQRD(); //RETURNS 1 IF BLACK IS DETECTED
int midQRD(); //RETURNS 1 IF BLACK IS DETECTED
int leftQRD(); //RETURNS 1 IF BLACK IS DETECTED

//INTERRUPTS
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void);

int main(void) {
    
    setupSteppers();
    setupQRDs();
    setupTimer();
    
    //SET UP PARAMETERS FOR STATE MACHINE
    enum {STRAIGHT, SLIGHTRIGHT, SLIGHTLEFT, HARDRIGHT, HARDLEFT, SEARCH} state;
    state = STRAIGHT;
    driveStraight();
    
    while(1){
        switch(state){
            
            case STRAIGHT:
                
                if(leftQRD()){
                    slightRight();
                    state = SLIGHTRIGHT;
                }
                
                break;
                
            case SLIGHTRIGHT:
                
                if(!leftQRD()){
                    driveStraight();
                    state = STRAIGHT;
                }
                
                break;
                
            case SLIGHTLEFT:
                
                break;
                
            case HARDRIGHT:
                
                break;
                
            case HARDLEFT:
                
                break;
                
            case SEARCH:
                
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

void setupSteppers(){
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
void setupTimer(){
    _TON = 1;       // Turn Timer1 on
    _TCKPS = 0b01;  // Chose pre-scaling as 8
    _TCS = 0;       // Internal clock source (FOSC/2)
    TMR1 = 0;       // Reset Timer1
}
void setupQRDs(){
    //RIGHT NOW WE ARE PLANNING ON USING DIGITAL PINS FOR THE QRDS BUT WE 
    //MIGHT HAVE TO SWITCH TO ANALOG IF THEY CONTINUE TO GIVE ME TROUBLE
    
    //RIGHT QRD
    _TRISA3 = 1;
    _ANSA3 = 0;
    
    //MID QRD
    _TRISB4 = 1;
    _ANSB4 = 0;
    
    //LEFT QRD
    _TRISA4 = 1;
    
}

void tankTurn(int degrees, int dir){
    
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
void driveStraight(){
    
    //SET PERIOD AND DUTY CYCLE
    OC2RS = FAST;
    OC2R = FAST/2;
    OC3RS = FAST;
    OC3R = FAST/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1;
    _LATA1 = 0;
    
}
void slightRight(){
    
    //SET PERIOD AND DUTY CYCLE
    //RIGHT
    OC2RS = 0;
    OC2R = 0/2;
    //LEFT
    OC3RS = FAST;
    OC3R = FAST/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1; //RIGHT
    _LATA1 = 0; //LEFT
    
}
void slightLeft(){
    //TODO: Define this function
}
void hardRight(){
    //TODO: Define this function
}
void hardLeft(){
    //TODO: Define this function
}
void search(){
    //TODO: Define this function
}

int rightQRD(){
    return _RA3;
}
int midQRD(){
    return _RB4;
}
int leftQRD(){
    return _RA4;
}