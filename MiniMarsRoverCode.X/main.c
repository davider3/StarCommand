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
#define FAST 75 //TODO:TRIAL AND ERROR TO DECIDE THE BEST VALUE FOR SPEED
#define SORTAFAST 100
#define SLOW 260
#define THRESHOLD 2500
#define LIMIT 300
#define FRONTSENSOR !_RB8

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

//FUNCTION PROTOTYPES

//SETUP FUNCTIONS
void setupSteppers();
void setupQRDs();
void setupTimer();
void setupDistanceSensors();
//FINITE STATE MACHINES
void lineFollowingFSM();
void taskDetectionFSM();
void canyonNavigationFSM();
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
int taskdetectionQRD(); //RETURNS 1 IF BLACK IS DETECTED
//INTERRUPTS
void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _OC3Interrupt(void);



int main(void) {
    
    setupSteppers();
    setupQRDs();
    setupTimer();
    setupDistanceSensors();
    
    //SET UP PARAMETERS FOR LINE FOLLOWING STATE MACHINE
    lineFollowingState = STRAIGHT;
    driveStraight();
    
    //SET UP PARAMETERS FOR TASK DETECTION STATE MACHINE
    taskDetectionState = TASKDETECTIONDEFAULT;
    
    //SET UP PARAMETERS FOR CANYON NAVIGATION STATE MACHINE
    canyonState = GOSTRAIGHT;
    
    //SET UP LED FOR DEBUGGING
    _TRISB7 = 0;
    
    
   
    while(1){     
        
        //lineFollowingFSM();
        //taskDetectionFSM();
        canyonNavigationFSM();
        
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
//SETUP FUNCTION DEFINITIONS
void setupSteppers(){
    //RIGHT, uses OC2, pin 4, RB0
    OC2CON1 = 0;
    OC2CON2 = 0;
    OC2CON1bits.OCTSEL = 0b111;
    OC2CON2bits.SYNCSEL = 0x1F;
    OC2CON2bits.OCTRIG = 0;
    OC2CON1bits.OCM = 0b110;
    
    _OC2IP = 4; // Select OC2 interrupt priority
    _OC2IE = 1; // Enable OC2 interrupt
    _OC2IF = 0; // Clear OC2 interrupt flag
    
    //LEFT, uses OC3, pin 5, RB1
    OC3CON1 = 0;
    OC3CON2 = 0;
    OC3CON1bits.OCTSEL = 0b111;
    OC3CON2bits.SYNCSEL = 0x1F;
    OC3CON2bits.OCTRIG = 0;
    OC3CON1bits.OCM = 0b110;
    
    _OC3IP = 4; // Select OC3 interrupt priority
    _OC3IE = 1; // Enable OC3 interrupt
    _OC3IF = 0; // Clear OC3 interrupt flag
    
    //SET UP DIRECTION PINS
    _TRISA0 = 0;
    _ANSA0 = 0; //this is unnecessary
    _ANSA1 = 0; //this is unnecessary
    _TRISA1 = 0;  
    
}
void setupTimer(){
    _TON = 1;       // Turn Timer1 on
    _TCKPS = 0b01;  // Chose pre-scaling as 8
    _TCS = 0;       // Internal clock source (FOSC/2)
    TMR1 = 0;       // Reset Timer1
}
void setupQRDs(){
    //SETUP THE ADC 
     
    _ADON = 0;    // Disable A/D module during configuration
    
    // AD1CON1
    _MODE12 = 1;  // 12-bit resolution
    _FORM = 0;    // unsigned integer output
    _SSRC = 7;    // auto convert
    _ASAM = 1;    // auto sample

    // AD1CON2
    _PVCFG = 0;   // use VDD as positive reference
    _NVCFG = 0;   // use VSS as negative reference
    _BUFREGEN = 1;// store results in buffer corresponding to channel number
    _CSCNA = 1;   // scanning mode
    _SMPI = 3;    // NUMBER OF ANALOG PINS MINUS ONE
    _ALTS = 0;    // sample MUXA only

    // AD1CON3
    _ADRC = 0;    // use system clock
    _SAMC = 1;    // sample every A/D period
    _ADCS = 0x3F; // TAD = 64*TCY

    //RIGHT
    _TRISB2 = 1;
    _ANSB2 = 1;
    _CSS4 = 1;
    
    //MID
    _TRISA2 = 1;
    _ANSA2 = 1;
    _CSS13 = 1;
    
    //LEFT
    _TRISA3 = 1;
    _ANSA3 = 1;
    _CSS14 = 1;
    
    //TASKDETECTION
    _TRISB12 = 1;
    _ANSB12 = 1;
    _CSS12 = 1;
    
    //TURN ON ADC
    _ADON = 1;
    
}
void setupDistanceSensors(){
    //FRONT SENSOR
    _TRISB8 = 1;
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
//CONTROL FUNCTION DEFINITIONS
void tankTurn(int degrees, int dir){
    
    stepsToTake = turnCoeff * degrees;

    //TURN ON INTERRUPT
    _OC2IE = 1;
    
    OC2Steps = 0;
    
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
//CHECK STATE FUNCTION DEFINITIONS
int rightQRD(){
    int onOffr;
    if(ADC1BUF4 > THRESHOLD){
        onOffr = 1;
    }else{
        onOffr = 0;
    }
    
    return onOffr;
}
int midQRD(){
    int onOffm;
    if(ADC1BUF13 > THRESHOLD){
        onOffm = 1;
    }else{
        onOffm = 0;
    }
    
    return onOffm;
}
int leftQRD(){
    int onOffl;
    if(ADC1BUF14 > THRESHOLD){
        onOffl = 1;
    }else{
        onOffl = 0;
    }
    
    return onOffl;
}
int taskdetectionQRD(){
    int onOfft;
    if(ADC1BUF12 > THRESHOLD){
        onOfft = 1;
    }else{
        onOfft = 0;
    }
    
    return onOfft;
}