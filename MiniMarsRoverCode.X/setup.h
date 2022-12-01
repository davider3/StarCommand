
#ifndef SETUP_H
#define	SETUP_H

#include <xc.h> 

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
    _TCKPS = 0b11;  // Chose pre-scaling as 256
    _TCS = 0;       // Internal clock source (FOSC/2)
    TMR1 = 0;       // Reset Timer1
}

void setupADC(){
     
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
    _SMPI = 5;    // NUMBER OF ANALOG PINS MINUS ONE
    _ALTS = 0;    // sample MUXA only

    // AD1CON3
    _ADRC = 0;    // use system clock
    _SAMC = 1;    // sample every A/D period
    _ADCS = 0x3F; // TAD = 64*TCY

}

void setupQRDs(){
    
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
    
    //BALLDETECTION
    _TRISB13 = 1;
    _ANSB13 = 1;
    _CSS11 = 1;
        
}

void turnOnADC(){
    _ADON = 1;
}

void setupDistanceSensors(){
    //FRONT SENSOR
    _TRISB8 = 1;
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

void setupDebugLED(){
    _TRISB7 = 0;
}

void setupPhotodiode(){
    _TRISB15 = 1;
    _ANSB15 = 1;
    _CSS9 = 1;
}

void setupLaser(){
    _TRISB8 = 0;
}


#endif	// SETUP_H

