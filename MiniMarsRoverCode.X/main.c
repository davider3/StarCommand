/*
 * File:   main.c
 * Author: Star Command Mechatronics Team
 *
 * Created on October 19, 2022, 6:25 PM
 */


#include "xc.h"
#include "setup.h"
#include "checkState.h"
#include "control.h"
#pragma config FNOSC = LPRC //31 kHz oscillator

#define CW 0
#define CCW 1
#define REST 0
#define ONEREV 200
#define ONESEC 1938
#define WHEELDIAMETER 69.5 //mm
#define TRACKWIDTH 221 //mm
#define FAST 75
#define SORTAFAST 100
#define SLOW 260
#define FRONTSENSOR !_RB8
#define LIMIT 300

//GLOBAL VARIABLES
int OC1Steps = 0;
int OC2Steps = 0;
int OC3Steps = 0;
float turnCoeff = TRACKWIDTH / (1.8 * WHEELDIAMETER); //1.7666
int stepsToTake = 0;
int lineCount = 0;

//FSM VARIABLES
enum {STRAIGHT, SLIGHTRIGHT, SLIGHTLEFT, HARDRIGHT, HARDLEFT} lineFollowingState;
enum {TASKDETECTIONDEFAULT, TASKDETECTIONBLACK, TASKDETECTIONWHITE} taskDetectionState;
enum {GOSTRAIGHT, WALLDETECTED} canyonState;
enum {WAIT, WHITEBALL, BLACKBALL} sampleState;

//FUNCTION PROTOTYPES
//FINITE STATE MACHINES
void lineFollowingFSM();
void taskDetectionFSM();
void canyonNavigationFSM();
void sampleReturnFSM();
//CONTROL FUNCTIONS
void tankTurn(int degrees, int dir);
//INTERRUPTS
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void);



int main(void) {
    
    setupSteppers();
    setupQRDs();
    setupTimer();
    setupDistanceSensors();
    setupServo();
    
    //SET UP PARAMETERS FOR LINE FOLLOWING STATE MACHINE
    lineFollowingState = STRAIGHT;
    driveStraight();
    
    //SET UP PARAMETERS FOR TASK DETECTION STATE MACHINE
    taskDetectionState = TASKDETECTIONDEFAULT;
    
    //SET UP PARAMETERS FOR CANYON NAVIGATION STATE MACHINE
    canyonState = GOSTRAIGHT;
    
    //SET UP PARAMETERS FOR SAMPLE RETURN STATE MACHINE
    sampleState = WAIT;
    closeGate();
    
    //SET UP LED FOR DEBUGGING
    _TRISB7 = 0;
    

   
    while(1){     
        
//        lineFollowingFSM();
//        taskDetectionFSM();
//        canyonNavigationFSM();
        sampleReturnFSM();
        
    }
    
    return 0;
}



//INTERRUPTS
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

//FSM FUNCTION DEFINITIONS
void lineFollowingFSM(){
    //LINE FOLLOWING FSM
        switch(lineFollowingState){
            
            case STRAIGHT:
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(midQRD()){
                        slightRight();
                        lineFollowingState = SLIGHTRIGHT;
                    }else{
                        hardRight();
                        lineFollowingState = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD is BLACK
                    if(midQRD()){
                        slightLeft();
                        lineFollowingState = SLIGHTLEFT;
                    }else{
                        hardLeft();
                        lineFollowingState = HARDLEFT;
                    }
                }
                break;
                
                
            case SLIGHTRIGHT:
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(!midQRD()){
                        hardRight();
                        lineFollowingState = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD is BLACK
                    if(midQRD()){
                        slightLeft();
                        lineFollowingState = SLIGHTLEFT;
                    }else{
                        hardLeft();
                        lineFollowingState = HARDLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight();
                    lineFollowingState = STRAIGHT;
                }
                break;
                
                
            case SLIGHTLEFT:
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(midQRD()){
                        slightRight();
                        lineFollowingState = SLIGHTRIGHT;
                    }else{
                        hardRight();
                        lineFollowingState = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD is BLACK
                    if(!midQRD()){
                        hardLeft();
                        lineFollowingState = HARDLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight();
                    lineFollowingState = STRAIGHT;
                }
                break;
                
                
            case HARDRIGHT:
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(midQRD()){
                        slightRight();
                        lineFollowingState = SLIGHTRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD IS BLACK
                    if(midQRD()){
                        slightLeft();
                        lineFollowingState = SLIGHTLEFT;
                    }else{
                        hardLeft();
                        lineFollowingState = HARDLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight();
                    lineFollowingState = STRAIGHT;
                }
                break;
                
                
            case HARDLEFT:
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(midQRD()){
                        slightRight();
                        lineFollowingState = SLIGHTRIGHT;
                    }else{
                        hardRight();
                        lineFollowingState = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD IS BLACK
                    if(midQRD()){
                        slightLeft();
                        lineFollowingState = SLIGHTLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight();
                    lineFollowingState = STRAIGHT;
                }
                break;
                
                
        }
}
void taskDetectionFSM(){
    switch(taskDetectionState){
        
            case TASKDETECTIONDEFAULT:
                if(taskdetectionQRD()){ //IF TASK DETECTION IS BLACK
                    taskDetectionState = TASKDETECTIONBLACK;
                    OC2Steps = 0;
                    _OC2IE = 1;
                    lineCount = 0;
                }
            break;
            
            case TASKDETECTIONBLACK:
                if(OC2Steps >= LIMIT){
                    taskDetectionState = TASKDETECTIONDEFAULT;
                    _OC2IE = 0;
                }
                else if(!taskdetectionQRD()){ //IF TASK DETECTION IS WHITE
                    lineCount++;
                    OC2Steps = 0;
                    taskDetectionState = TASKDETECTIONWHITE;
                }
            break;
            
            case TASKDETECTIONWHITE:
                if(OC2Steps >= LIMIT){
                    taskDetectionState = TASKDETECTIONDEFAULT;
                    _OC2IE = 0;
                }
                else if(lineCount == 3){
                    OC2R = 0;
                    OC3R = 0;
                }
                else if(taskdetectionQRD()){ //IF TASK DETECTION IS BLACK
                    taskDetectionState = TASKDETECTIONBLACK;
                    _OC2IE = 0;
                }
            break;
        }
}
void canyonNavigationFSM(){
    switch(canyonState){
        case GOSTRAIGHT:
            if(FRONTSENSOR){
                canyonState = WALLDETECTED;
                OC2Steps = 0;
                tankTurn(90, CW);
            }
            break;
            
        case WALLDETECTED:
            if(OC2Steps >= stepsToTake){
                canyonState = GOSTRAIGHT;
                driveStraight();
            }
            break;
    }
}
void sampleReturnFSM(){
    switch(sampleState){
        case WAIT:
            if(OC2Steps >= stepsToTake){
               stop();
               OC2Steps = 0;
            }
            if(TMR1 > 3*ONESEC){
                if(ballQRD()){
                    sampleState = BLACKBALL;
                    tankTurn(90, CW);
                    TMR1 = 0;
                }else if(!ballQRD()){
                    sampleState = WHITEBALL;
                    tankTurn(90, CCW);
                    TMR1 = 0;
                }
            }
            break;
            
        case WHITEBALL:
            if(OC2Steps >= stepsToTake){
                openGate();
                OC2Steps = 0;
                stop();
            }else if(TMR1 > 3*ONESEC){
                sampleState = WAIT;
                tankTurn(90, CW);
                closeGate();
                TMR1 = 0;
            }
            break;
            
        case BLACKBALL:
            if(OC2Steps >= stepsToTake){
                openGate();
                OC2Steps = 0;
                stop();
            }else if(TMR1 > 3*ONESEC){
                sampleState = WAIT;
                tankTurn(90, CCW);
                closeGate();
                TMR1 = 0;
            }
            break;
    }
}

//CONTROL FUNCTION DEFINITIONS
void tankTurn(int degrees, int dir){
    
    stepsToTake = turnCoeff * degrees;

    //TURN ON INTERRUPT
    _OC2IE = 1;
    
    OC2Steps = 0;
    
    //SET PERIOD AND DUTY CYCLE
    OC2RS = SORTAFAST;
    OC2R = SORTAFAST/2;
    OC3RS = SORTAFAST;
    OC3R = SORTAFAST/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = dir;
    _LATA1 = dir;
}

