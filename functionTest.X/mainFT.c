/*
 * File:   mainFT.c
 * Author: drein
 *
 * Created on November 10, 2022, 8:37 AM
 */


#include "xc.h"
#include "LED.h"
#pragma config FNOSC = LPRC


#define ONESEC 500

void setupTimer();


int main(void) {
    
    setupTimer();
    
    _TRISB7 = 0;
    
    turnOnLED();
    
    while(1){
        if(TMR1 > 2*ONESEC){
            turnOnLED();
            TMR1 = 0;
        }else if(TMR1 > ONESEC){
            turnOffLED();
        }
    }
    
    return 0;
}

void setupTimer(){
    _TON = 1;       // Turn Timer1 on
    _TCKPS = 0b01;  // Chose pre-scaling as 8
    _TCS = 0;       // Internal clock source (FOSC/2)
    TMR1 = 0;       // Reset Timer1
}