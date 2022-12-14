#ifndef CHECK_STATE_H
#define	CHECK_STATE_H

#include <xc.h> 
#define THRESHOLD 2000
#define TASKDETECT 3200
#define CANYONTHRESHOLD 3000
#define LANDERTHRESHOLD 2800

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
    if(ADC1BUF12 > TASKDETECT){
        onOfft = 1;
    }else{
        onOfft = 0;
    }
    
    return onOfft;
}

int ballQRD(){
   int onOffb;
    if(ADC1BUF11 > 2500){
        onOffb = 1;
    }else{
        onOffb = 0;
    }
    
    return onOffb; 
}

int landerQRD(){
   int onOffb;
    if(ADC1BUF10 > LANDERTHRESHOLD){
        onOffb = 1;
    }else{
        onOffb = 0;
    }
    
    return onOffb; 
}

int exitCanyonQRD(){
    int onOffm;
    if(ADC1BUF13 > CANYONTHRESHOLD){
        onOffm = 1;
    }else{
        onOffm = 0;
    }
    
    return onOffm;
}

int photodiode(){
    return ADC1BUF9;
}

#endif	/* CHECK_STATE_H */

