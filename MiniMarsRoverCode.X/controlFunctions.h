#ifndef CONTROL_FUNCTIONS_H
#define	CONTROL_FUNCTIONS_H

#include <xc.h> 
#include "checkState.h"
#define STARTSPEED 4000
#define FAST 3000 //3000 for fast, 8000 for canyon
#define CANYONSPEED 6000
#define SORTAFAST 4000
#define SLOW 20000
#define OPENSERVO 30 //TODO: Change for new oscillator
#define CLOSESERVO 10
#define CATCHSERVO 18
#define SERVOPERIOD 390
#define FILTERWEIGHT 0.1
#define ONESEC 15625

void straightStart(){
    
    //SET PERIOD AND DUTY CYCLE
    OC2RS = STARTSPEED;
    OC2R = STARTSPEED/2;
    OC3RS = STARTSPEED;
    OC3R = STARTSPEED/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1;
    _LATA1 = 0;
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

void canyonStraight(){
    
    //SET PERIOD AND DUTY CYCLE
    OC2RS = CANYONSPEED;
    OC2R = CANYONSPEED/2;
    OC3RS = CANYONSPEED;
    OC3R = CANYONSPEED/2;
    
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

void search(int prev){
    if(prev){
        hardRight();
    }else{
        hardLeft();
    }
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

void catchBall(){
    OC1RS = SERVOPERIOD;
    OC1R = CATCHSERVO;
}

void stopServo(){
    OC1RS = 0;
    OC1R = 0;
}

void turnOnLaser(){
    _LATB8 = 1;
}

void turnOffLaser(){
    _LATB8 = 0;
}

void IRSearch(){
    //Servo opens door to horizontal
    if(OC1R > CLOSESERVO){
        if(TMR1 > 2000){
        OC1R = OC1R - 0.1;
        TMR1 = 0;
    }
    }
    else{
        OC1R = 25;
    }
}

void adjRL(){
    
    //DECLARE LOCAL VARIBLES
    int TAdjRight;
    int TAdjLeft;
    int adjFAST = FAST*0.7;
    //RIGHT
    TAdjRight = adjFAST + (SLOW - adjFAST)/4096*ADC1BUF4;
    OC2RS = TAdjRight;
    OC2R = TAdjRight/2;
    
    //LEFT
    TAdjLeft = adjFAST + (SLOW - adjFAST)/4096*ADC1BUF14;
    OC3RS = TAdjLeft;
    OC3R = TAdjLeft/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1; //RIGHT
    _LATA1 = 0; //LEFT
    
}

void moveServo(float dutyCycle){
    OC1RS = SERVOPERIOD;
    OC1R = dutyCycle;
}

void findSat(){
    int servoAngle = 25;
    int bestAngle = 25;
    int maxSat = 0;
    TMR1 = 0;
    while(servoAngle >= CLOSESERVO){
        moveServo(servoAngle);
        if(TMR1 >= .3*ONESEC){
            TMR1 = 0;
            if(photodiode() > maxSat){
                maxSat = photodiode();
                bestAngle = servoAngle;
            }
            servoAngle -= .2;
        }
    }
    
    moveServo(bestAngle);
}




#endif	/* CONTROL_FUNCTIONS_H */

