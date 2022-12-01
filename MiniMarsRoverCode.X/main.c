/*
 * File:   main.c
 * Author: Star Command Mechatronics Team
 *
 * Created on October 19, 2022, 6:25 PM
 */


#include "xc.h"
#include "setup.h"
#include "checkState.h"
#include "controlFunctions.h"
#pragma config FNOSC = FRC //8 MHz oscillator

#define CW 0
#define CCW 1
#define ONEREV 200
#define ONESEC 1938 //TODO: Change for new Timer
#define TANKTURNSPEED 100
#define FRONTSENSOR !_RB8
#define LIMIT 300

//GLOBAL VARIABLES
int OC1Steps = 0;
int OC2Steps = 0;
int OC3Steps = 0;
float turnCoeff = 1.7666; //TRACKWIDTH/(1.8 * WHEELDIAMETER)
int stepsToTake = 0;
int lineCount = 0;
int prevState = 0;

//FSM VARIABLES
enum {STRAIGHT, SLIGHTRIGHT, SLIGHTLEFT, HARDRIGHT, HARDLEFT, SEARCH} lineFollowingState;
enum {ADJUST, GORIGHT, GOLEFT} lineFollowingState2;
enum {TASKDETECTIONDEFAULT, TASKDETECTIONBLACK, TASKDETECTIONWHITE} taskDetectionState;
enum {GOSTRAIGHT, WALLDETECTED} canyonState;
enum {WAIT, WHITEBALL, BLACKBALL} sampleState;

//FUNCTION PROTOTYPES
//FINITE STATE MACHINES
void lineFollowingFSM();
void lineFollowingFSM2();
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
    setupADC();
    setupPhotodiode();
    setupQRDs();
    setupTimer();
    setupDistanceSensors();
    setupServo();
    setupDebugLED();
    turnOnADC();
    setupLaser();
    stop();
  
    //SET UP PARAMETERS FOR STATE MACHINES
    lineFollowingState = STRAIGHT;
    lineFollowingState2 = ADJUST;
    driveStraight();
    taskDetectionState = TASKDETECTIONDEFAULT;
    canyonState = GOSTRAIGHT;
    sampleState = WAIT;

    OC1RS = SERVOPERIOD;
    OC1R = 25;
    closeGate();
    
    while(1){    
//        if(photodiode() < 500){
//            turnOffLaser(); 
//            IRSearch();
//        }else{
//            turnOnLaser();
//        }
        lineFollowingFSM();
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
                prevState = 1;
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
                else{
                    lineFollowingState = SEARCH;
                    search(prevState);
                }
                break;
                
                
            case SLIGHTLEFT:
                prevState = 0;
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
                else{
                    lineFollowingState = SEARCH;
                    search(prevState);
                }
                break;
                
                
            case HARDRIGHT:
                prevState = 1;
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
                else{
                    lineFollowingState = SEARCH;
                    search(prevState);
                }
                break;
                
                
            case HARDLEFT:
                prevState = 0;
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
                else{
                    lineFollowingState = SEARCH;
                    search(prevState);
                }
                break;
                
            case SEARCH:
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
                    }else{
                        hardLeft();
                        lineFollowingState = HARDLEFT;
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
void lineFollowingFSM2(){
    //LINE FOLLOWING FSM
        switch(lineFollowingState2){
            
            case ADJUST: //ADJUSTMENT State
                adjRL();
                if(!midQRD()) //IF CENTER QRD IS WHITE
                /*{
                    if(rightQRD()) //IF RIGHT QRD IS BLACK
                    {
                        hardRight();
                        lineFollowingState2 = GORIGHT;
                    }
                    else if(leftQRD()){  //IF LEFT QRD IS BLACK
                        hardLeft();
                        lineFollowingState2 = GOLEFT;
                    }
                }*/
                break;
                
            case GORIGHT:
                hardRight();
                if(midQRD()){ //IF MIDDLE IS BLACK
                    adjRL();
                    lineFollowingState2 = ADJUST;
                }
            
                
                break;
                
                
            case GOLEFT:
                prevState = 0;
                hardLeft();
                if(midQRD()){ //IF MIDDLE IS BLACK
                    adjRL();
                    lineFollowingState2 = ADJUST;
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
    OC2RS = TANKTURNSPEED;
    OC2R = TANKTURNSPEED/2;
    OC3RS = TANKTURNSPEED;
    OC3R = TANKTURNSPEED/2;
    
    //WRITE TO DIRECTION PINS
    _LATA0 = dir;
    _LATA1 = dir;
}

