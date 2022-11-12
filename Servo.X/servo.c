/*
 * File:   servo.c
 * Author: drein
 *
 * Created on November 11, 2022, 8:51 PM
 */


#include "xc.h"
#pragma config FNOSC = LPRC
#define THRESHOLD 2500
#define OPENSERVO 30
#define CLOSESERVO 10
#define SERVOPERIOD 387

void setupServo();
void setupQRD();
void openGate();
void closeGate();
int ballQRD();

int main(void) {
    setupServo();
    setupQRD();
    
    while(1){
        if(ballQRD()){
            openGate();
        }else{
            closeGate();
        }
    }
    
    return 0;
}

void setupServo(){
    //PIN 14
    OC1CON1 = 0;
    OC1CON2 = 0;
    OC1CON1bits.OCTSEL = 0b111;
    OC1CON2bits.SYNCSEL = 0x1F;
    OC1CON2bits.OCTRIG = 0;
    OC1CON1bits.OCM = 0b110;
    
    _OC1IP = 4; // Select OC1 interrupt priority
    _OC1IE = 0; // Disable OC1 interrupt
    
    OC1RS = 0;
    OC1R = 0;
}
void setupQRD(){
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
    _SMPI = 0;    // NUMBER OF ANALOG PINS MINUS ONE
    _ALTS = 0;    // sample MUXA only

    // AD1CON3
    _ADRC = 0;    // use system clock
    _SAMC = 1;    // sample every A/D period
    _ADCS = 0x3F; // TAD = 64*TCY

    //BALLDETECTION
    _TRISB13 = 1;
    _ANSB13 = 1;
    _CSS11 = 1;
    
    //TURN ON ADC
    _ADON = 1;
    
}

void openGate(){
    OC1RS = SERVOPERIOD;
    OC1R = OPENSERVO;
}
void closeGate(){
    OC1RS = SERVOPERIOD;
    OC1R = CLOSESERVO;
}

int ballQRD(){
   int onOffb;
    if(ADC1BUF11 > THRESHOLD){
        onOffb = 1;
    }else{
        onOffb = 0;
    }
    
    return onOffb; 
}