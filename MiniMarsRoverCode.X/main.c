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
#define ONESEC 15625
#define TANKTURNSPEED 6000
#define FORWARDSPEED 6000
#define FRONTSENSOR !_RB8
#define LIMIT 100
#define LANDERDISTANCE 620 //mm
#define APPROACHDIS 240 //mm

//GLOBAL VARIABLES
int OC1Steps = 0;
int OC2Steps = 0;
int OC3Steps = 0;
float turnCoeff = 3.65; //TRACKWIDTH/(.9 * WHEELDIAMETER)
float forwardCoeff = 2.15; //400 / (PI * WHEELDIAMETER))
int stepsToTake = 0;
int lineCount = 0;
int nextTask = 2;
int prevState = 0;
int finished = 0; //0 NOT DONE, 1 DONE

//FSM VARIABLES
enum{START, LINEFOLLOW, GETBALL, SAMPLERETURN, CANYONNAVIGATION} roveState;
enum {STRAIGHT, SLIGHTRIGHT, SLIGHTLEFT, HARDRIGHT, HARDLEFT, SEARCH} lineFollowingState;
enum {ADJUST, GORIGHT, GOLEFT} lineFollowingState2;
enum {TASKDETECTIONDEFAULT, TASKDETECTIONBLACK, TASKDETECTIONWHITE} taskDetectionState;
enum {GOSTRAIGHT, WALLDETECTED} canyonState;
enum {WAIT, WHITEBALL, BLACKBALL} sampleState;
enum {FORWARD, TURN} startState;
enum {ALIGN, DELAY, RIGHT, APPROACH, CATCH, BACKUP, LEFT} getBallState;

//FUNCTION PROTOTYPES
//FINITE STATE MACHINES
void roveFSM();
void lineFollowingFSM();
void lineFollowingFSM2();
void lineFollowingFSM3();
void taskDetectionFSM();
void canyonNavigationFSM();
void sampleReturnFSM();
void startFSM();
void getBallFSM();
//CONTROL FUNCTIONS
void tankTurn(int degrees, int dir);
void goForward(int mmDis, int dir);
//INTERRUPTS
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void);



//MAIN FUNCTION
int main(void) {
    
    setupSteppers();
    stop();
    setupADC();
    setupPhotodiode();
    setupQRDs();
    setupTimer();
    setupDistanceSensors();
    setupServo();
    setupDebugLED();
    turnOnADC();
    setupLaser();
  
    //SET UP PARAMETERS FOR STATE MACHINES
    roveState = LINEFOLLOW;
    lineFollowingState = STRAIGHT;
    driveStraight();
    taskDetectionState = TASKDETECTIONDEFAULT;
    canyonState = GOSTRAIGHT;
    sampleState = WAIT;
    startState = FORWARD;
    getBallState = ALIGN;
    
   // startFSM
   // goForward(LANDERDISTANCE, 1);
    
    
    while(1){
        roveFSM();
//        if(taskdetectionQRD()){
//            debugLED(1);
//        }else debugLED(0);
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
void roveFSM(){
    
    switch(roveState){
        case START:
            startFSM();
            if(finished){
                finished = 0;
                roveState = LINEFOLLOW;
            }
            break;
            
        case LINEFOLLOW:
            lineFollowingFSM();
            taskDetectionFSM();
            if(finished){
                finished = 0;
                lineCount = 0;
                if(nextTask == 2){
                    roveState = GETBALL;
                    ++nextTask;
                    driveStraight();
                    TMR1 = 0;
                }else if(nextTask == 3){
                    ++nextTask;
                    //RETURN BALL
                }else if(nextTask == 4){
                    //CANYON
                }
            }
            break;
            
        case GETBALL:
            getBallFSM();
            if(finished){
                finished = 0;
                roveState = LINEFOLLOW;
            }
            break;
            
        case SAMPLERETURN:
            sampleReturnFSM();
            if(finished){
                finished = 0;
                roveState = LINEFOLLOW;
            }
            break;
            
        case CANYONNAVIGATION:
            canyonNavigationFSM();
            if(finished){
                finished = 0;
                roveState = LINEFOLLOW;
            }
            break;
    }
}
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
                    OC3Steps = 0;
                    _OC3IE = 1;
                    lineCount = 0;
                }
            break;
            
            case TASKDETECTIONBLACK:
                if(OC3Steps >= LIMIT){
                    taskDetectionState = TASKDETECTIONDEFAULT;
                    _OC3IE = 0;
                }
                else if(!taskdetectionQRD()){ //IF TASK DETECTION IS WHITE
                    lineCount++;
                    OC3Steps = 0;
                    taskDetectionState = TASKDETECTIONWHITE;
                }
            break;
            
            case TASKDETECTIONWHITE:
                if(OC3Steps >= LIMIT){
                    taskDetectionState = TASKDETECTIONDEFAULT;
                    _OC3IE = 0;
                }
                else if(lineCount == nextTask){
                    finished = 1;
                    taskDetectionState = TASKDETECTIONDEFAULT;
//                    OC2R = 0;
//                    OC2RS = 0;
//                    OC3R = 0;
//                    OC3RS = 0;
                }
                else if(taskdetectionQRD()){ //IF TASK DETECTION IS BLACK
                    taskDetectionState = TASKDETECTIONBLACK;
                    _OC3IE = 0;
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
            if(TMR1 > ONESEC){
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
            }else if(TMR1 > ONESEC){
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
            }else if(TMR1 > ONESEC){
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
                /*if(!midQRD()) //IF CENTER QRD IS WHITE
                {
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
void lineFollowingFSM3(){
    //LINE FOLLOWING FSM
        switch(lineFollowingState){
            
            case STRAIGHT:
                driveStraight2();
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(midQRD()){
                        slightRight2();
                        lineFollowingState = SLIGHTRIGHT;
                    }else{
                        hardRight2();
                        lineFollowingState = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD is BLACK
                    if(midQRD()){
                        slightLeft2();
                        lineFollowingState = SLIGHTLEFT;
                    }else{
                        hardLeft2();
                        lineFollowingState = HARDLEFT;
                    }
                }
                break;
                
                
            case SLIGHTRIGHT:
                prevState = 1;
                slightRight2();
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(!midQRD()){
                        hardRight2();
                        lineFollowingState = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD is BLACK
                    if(midQRD()){
                        slightLeft2();
                        lineFollowingState = SLIGHTLEFT;
                    }else{
                        hardLeft2();
                        lineFollowingState = HARDLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight2();
                    lineFollowingState = STRAIGHT;
                }
                else{
                    lineFollowingState = SEARCH;
                    search(prevState);
                }
                break;
                
                
            case SLIGHTLEFT:
                prevState = 0;
                slightLeft2();
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(midQRD()){
                        slightRight2();
                        lineFollowingState = SLIGHTRIGHT;
                    }else{
                        hardRight2();
                        lineFollowingState = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD is BLACK
                    if(!midQRD()){
                        hardLeft2();
                        lineFollowingState = HARDLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight2();
                    lineFollowingState = STRAIGHT;
                }
                else{
                    lineFollowingState = SEARCH;
                    search(prevState);
                }
                break;
                
                
            case HARDRIGHT:
                prevState = 1;
                hardRight2();
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(midQRD()){
                        slightRight2();
                        lineFollowingState = SLIGHTRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD IS BLACK
                    if(midQRD()){
                        slightLeft2();
                        lineFollowingState = SLIGHTLEFT;
                    }else{
                        hardLeft2();
                        lineFollowingState = HARDLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight2();
                    lineFollowingState = STRAIGHT;
                }
                else{
                    lineFollowingState = SEARCH;
                    search(prevState);
                }
                break;
                
                
            case HARDLEFT:
                prevState = 0;
                hardLeft2();
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(midQRD()){
                        slightRight2();
                        lineFollowingState = SLIGHTRIGHT;
                    }else{
                        hardRight2();
                        lineFollowingState = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD IS BLACK
                    if(midQRD()){
                        slightLeft2();
                        lineFollowingState = SLIGHTLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight2();
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
                        slightRight2();
                        lineFollowingState = SLIGHTRIGHT;
                    }else{
                        hardRight2();
                        lineFollowingState = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD IS BLACK
                    if(midQRD()){
                        slightLeft2();
                        lineFollowingState = SLIGHTLEFT;
                    }else{
                        hardLeft2();
                        lineFollowingState = HARDLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight2();
                    lineFollowingState = STRAIGHT;
                }
            
            
                break;
                
                
        }
}
void startFSM(){
    switch(startState){
        
        case FORWARD:
            
            if(OC2Steps >= stepsToTake){
                tankTurn(90, CCW);
                startState = TURN;
            }
            
            break;
            
        case TURN:
            
            if(OC2Steps >= stepsToTake){
                finished = 1;
            }
            
            break;
        
    }  
}
void getBallFSM(){
    
    switch(getBallState){
        
        case ALIGN:
            
            if(TMR1 >= .4*ONESEC){
                getBallState = DELAY;
                stop();
                TMR1 = 0;
            }
            break;
            
        case DELAY:
            
            if(TMR1 > .2*ONESEC){
                getBallState = RIGHT;
                tankTurn(85, CW);
            } 
            break;
            
        case RIGHT:
            
            if(OC2Steps >= stepsToTake){
                getBallState = APPROACH;
                goForward(APPROACHDIS, 1);
            }
            break;
            
        case APPROACH:
            
            if(OC2Steps >= stepsToTake){
                stop();
                getBallState = CATCH;
                TMR1 = 0;
            }
            break;
            
        case CATCH:
            
            if(TMR1 > ONESEC ){
                goForward(APPROACHDIS, 0);
                getBallState = BACKUP;
            }
            break;
            
        case BACKUP:
            
            if(OC2Steps >= stepsToTake){
                tankTurn(80, CCW);
                getBallState = LEFT;
            }
            break;
            
        case LEFT:
            
            if(OC2Steps >= stepsToTake){
                finished = 1;
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
void goForward(int mmDis, int dir){
    
    stepsToTake = forwardCoeff * mmDis;
    
    OC2Steps = 0;
    
    _OC2IE = 1;
    
    //SET PERIOD AND DUTY CYCLE
    OC2RS = FORWARDSPEED;
    OC2R = FORWARDSPEED/2;
    OC3RS = FORWARDSPEED;
    OC3R = FORWARDSPEED/2;
    
    //WRITE TO DIRECTION PINS
    //1 IS FORWARD, 0 IS REVERSE
    if(dir){
        _LATA0 = 1; //GOING FORWARD
        _LATA1 = 0;
    }else{
        _LATA0 = 0; //GOING REVERSE
        _LATA1 = 1;
    }
}

