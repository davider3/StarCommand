#ifndef LED_H
#define LED_H

#include <xc.h>

void turnOnLED(){
    _LATB7 = 1;
}

void turnOffLED(){
    _LATB7 = 0;
}

#endif	/* XC_HEADER_TEMPLATE_H */

