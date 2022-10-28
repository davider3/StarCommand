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
#define FAST 50 //TODO:TRIAL AND ERROR TO DECIDE THE BEST VALUE FOR SPEED
#define SORTAFAST 70
#define SLOW 90
#define THRESHOLD 2000


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
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(midQRD()){
                        slightRight();
                        state = SLIGHTRIGHT;
                    }else{
                        hardRight();
                        state = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD is BLACK
                    if(midQRD()){
                        slightLeft();
                        state = SLIGHTLEFT;
                    }else{
                        hardLeft();
                        state = HARDLEFT;
                    }
                }else if(!midQRD()){ //IF ALL 3 ARE WHITE
                    search();
                    state = SEARCH;
                }
                break;
                
            case SLIGHTRIGHT:
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(!midQRD()){
                        hardRight();
                        state = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD is BLACK
                    if(midQRD()){
                        slightLeft();
                        state = SLIGHTLEFT;
                    }else{
                        hardLeft();
                        state = HARDLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight();
                    state = STRAIGHT;
                }else{
                    search();
                    state = SEARCH;
                }
                break;
                
            case SLIGHTLEFT:
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(midQRD()){
                        slightRight();
                        state = SLIGHTRIGHT;
                    }else{
                        hardRight();
                        state = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD is BLACK
                    if(!midQRD()){
                        hardLeft();
                        state = HARDLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight();
                    state = STRAIGHT;
                }else{
                    search();
                    state = SEARCH;
                }
                break;
                
            case HARDRIGHT:
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(midQRD()){
                        slightRight();
                        state = SLIGHTRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD IS BLACK
                    if(midQRD()){
                        slightLeft();
                        state = SLIGHTLEFT;
                    }else{
                        hardLeft();
                        state = HARDLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight();
                    state = STRAIGHT;
                }else{
                    search();
                    state = SEARCH;
                }
                break;
                
            case HARDLEFT:
                if(rightQRD()){ //RIGHTQRD IS BLACK
                    if(midQRD()){
                        slightRight();
                        state = SLIGHTRIGHT;
                    }else{
                        hardRight();
                        state = HARDRIGHT;
                    }
                }else if(leftQRD()){ //LEFTQRD IS BLACK
                    if(midQRD()){
                        slightLeft();
                        state = SLIGHTLEFT;
                    }
                }else if(midQRD()){ //IF MIDDLE IS BLACK
                    driveStraight();
                    state = STRAIGHT;
                }else{
                    search();
                    state = SEARCH;
                }
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
    _SMPI = 0;    // begin new sampling sequence after every sample
    _ALTS = 0;    // sample MUXA only

    // AD1CON3
    _ADRC = 0;    // use system clock
    _SAMC = 1;    // sample every A/D period
    _ADCS = 0x3F; // TAD = 64*TCY

    //RIGHT
    _CSS4 = 1; //set right to pin 6 or RB2
    
    //MID
    _CSS13 = 1; //set mid to pin 7 or RA2
    
    //LEFT
    _CSS14 = 1; //set left to pin 8 or RA3
    
    
    //TURN ON ADC
    _ADON = 1;
    
}

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

int rightQRD(){
    int onOffr;
    if(ADC1BUF4 > THRESHOLD){
        onOffr = 0;
    }else{
        onOffr = 1;
    }
    
    return onOffr;
}
int midQRD(){
    int onOffm;
    if(ADC1BUF13 > THRESHOLD){
        onOffm = 0;
    }else{
        onOffm = 1;
    }
    
    return onOffm;
}
int leftQRD(){
    int onOffl;
    if(ADC1BUF14 > THRESHOLD){
        onOffl = 0;
    }else{
        onOffl = 1;
    }
    
    return onOffl;
}