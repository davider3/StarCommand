#ifndef CONTROL_H
#define	CONTROL_H

#include <xc.h>
#include "globalVariables.h"

#define FAST 75 
#define SORTAFAST 100
#define SLOW 260
#define OPENSERVO 31
#define CLOSESERVO 15
#define SERVOPERIOD 387
#define WHEELDIAMETER 69.5 //mm
#define TRACKWIDTH 221 //mm


float turnCoeff = TRACKWIDTH / (1.8 * WHEELDIAMETER);


void tankTurn(int degrees, int dir){
    
    setStepsToTake(turnCoeff * degrees);

    //TURN ON INTERRUPT
    _OC2IE = 1;
    
    setOC2Steps(0);
    
    //SET PERIOD AND DUTY CYCLE
    OC2RS = FAST;
    OC2R = FAST/2;
    OC3RS = FAST;
    OC3R = FAST/2;
    
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
    OC2RS = SORTAFAST;
    OC2R = SORTAFAST/2;
    //LEFT
    OC3RS = FAST;
    OC3R = FAST/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1; //RIGHT
    _LATA1 = 0; //LEFT
    
}

void slightLeft(){
    
    //SET PERIOD AND DUTY CYCLE
    //RIGHT
    OC2RS = FAST;
    OC2R = FAST/2;
    //LEFT
    OC3RS = SORTAFAST;
    OC3R = SORTAFAST/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1; //RIGHT
    _LATA1 = 0; //LEFT
}

void hardRight(){
    
    //SET PERIOD AND DUTY CYCLE
    //RIGHT
    OC2RS = SLOW;
    OC2R = SLOW/2;
    //LEFT
    OC3RS = FAST;
    OC3R = FAST/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1; //RIGHT
    _LATA1 = 0; //LEFT
}

void hardLeft(){
    
    //SET PERIOD AND DUTY CYCLE
    //RIGHT
    OC2RS = FAST;
    OC2R = FAST/2;
    //LEFT
    OC3RS = SLOW;
    OC3R = SLOW/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1; //RIGHT
    _LATA1 = 0; //LEFT
}

void search(){
    //TODO: Define this function
}

void debugLED(int onOff){
    _LATB7 = onOff;
}

void openGate(){
    OC1RS = SERVOPERIOD;
    OC1R = OPENSERVO;
}

void closeGate(){
    OC1RS = SERVOPERIOD;
    OC1R = CLOSESERVO;
}

#endif	// CONTROL_H

