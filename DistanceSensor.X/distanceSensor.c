/*
 * File:   distanceSensor.c
 * Author: drein
 *
 * Created on November 2, 2022, 2:31 PM
 */


#include "xc.h"

int main(void) {
    
    //SET UP LED
    _TRISB7 = 0;
    
    //SET UP DISTANCE SENSOR
    _TRISB8 = 1;
    
    
    while(1){
        if(_RB8){
            _LATB7 = 1;
        }else{
            _LATB7 = 0;
        }
    }
    
    
    return 0;
}
