#ifndef CONTROL_H
#define	CONTROL_H

#include <xc.h> 
#define FAST 75
#define SORTAFAST 100
#define SLOW 260
#define OPENSERVO 30
#define CLOSESERVO 10
#define SERVOPERIOD 387

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

void stop(){
    
     //SET PERIOD AND DUTY CYCLE
    //RIGHT
    OC2RS = 0;
    OC2R = 0;
    //LEFT
    OC3RS = 0;
    OC3R = 0;
    
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

void turnOnLaser(){
    _LATB8 = 1;
}

void turnOffLaser(){
    _LATB8 = 0;
}

#endif	/* CONTROL_H */

