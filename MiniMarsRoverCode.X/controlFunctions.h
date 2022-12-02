#ifndef CONTROL_FUNCTIONS_H
#define	CONTROL_FUNCTIONS_H

#include <xc.h> 
#define FAST 3000
#define SORTAFAST 4000
#define SLOW 20000
#define OPENSERVO 30 //TODO: Change for new oscillator
#define CLOSESERVO 10
#define SERVOPERIOD 387
#define FILTERWEIGHT 0.1

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

void turnOnLaser(){
    _LATB8 = 1;
}

void turnOffLaser(){
    _LATB8 = 0;
}

void IRSearch(){
    //Servo opens door to horizontal
    if(OC1R > CLOSESERVO){
        if(TMR1 > 400){
        OC1R = OC1R - 0.5;
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

void driveStraight2(){
    
    //SET PERIOD AND DUTY CYCLE
    OC2RS = (1-FILTERWEIGHT)*OC2RS + FILTERWEIGHT*FAST;
    OC2R = OC2RS/2;
    OC3RS = (1-FILTERWEIGHT)*OC3RS + FILTERWEIGHT*FAST;
    OC3R = OC3RS/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1;
    _LATA1 = 0;
    
}

void slightRight2(){
    
    //SET PERIOD AND DUTY CYCLE
    //RIGHT
    OC2RS = (1-FILTERWEIGHT)*OC2RS + FILTERWEIGHT*SORTAFAST;
    OC2R = OC2RS/2;
    //LEFT
    OC3RS = (1-FILTERWEIGHT)*OC3RS + FILTERWEIGHT*FAST;
    OC3R = OC3RS/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1; //RIGHT
    _LATA1 = 0; //LEFT
    
}

void slightLeft2(){
    
    //SET PERIOD AND DUTY CYCLE
    //RIGHT
    OC2RS = (1-FILTERWEIGHT)*OC2RS + FILTERWEIGHT*FAST;
    OC2R = OC2RS/2;
    //LEFT
    OC3RS = (1-FILTERWEIGHT)*OC3RS + FILTERWEIGHT*SORTAFAST;
    OC3R = OC3RS/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1; //RIGHT
    _LATA1 = 0; //LEFT
}

void hardRight2(){
    
    //SET PERIOD AND DUTY CYCLE
    //RIGHT
    OC2RS = (1-FILTERWEIGHT)*OC2RS + FILTERWEIGHT*SLOW;
    OC2R = OC2RS/2;
    //LEFT
    OC3RS = (1-FILTERWEIGHT)*OC3RS + FILTERWEIGHT*FAST;
    OC3R = OC3RS/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1; //RIGHT
    _LATA1 = 0; //LEFT
}

void hardLeft2(){
    
    //SET PERIOD AND DUTY CYCLE
    //RIGHT
    OC2RS = (1-FILTERWEIGHT)*OC2RS + FILTERWEIGHT*FAST;
    OC2R = OC2RS/2;
    //LEFT
    OC3RS = (1-FILTERWEIGHT)*OC3RS + FILTERWEIGHT*SLOW;
    OC3R = OC3RS/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = 1; //RIGHT
    _LATA1 = 0; //LEFT
}



#endif	/* CONTROL_FUNCTIONS_H */

