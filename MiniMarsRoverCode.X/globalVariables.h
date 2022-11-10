 
#ifndef GLOBAL_VARIABLES_H
#define	GLOBAL_VARIABLES_H

#include <xc.h>

int OC1Steps = 0;
int OC2Steps = 0;
int OC3Steps = 0; //1.7666
int stepsToTake = 0;
int lineCount = 0;

int getOC2Steps(){
    return OC2Steps;
}

void setOC2Steps(int steps){
    OC2Steps = steps;
}

int getStepsToTake(){
    return stepsToTake;
}

void setStepsToTake(int steps){
    stepsToTake = steps;
}

int getLineCount(){
    return lineCount;
}

void setLineCount(int count){
    lineCount = count;
}

void incrementLineCount(){
    ++lineCount;
}

#endif	/* GLOBAL_VARIABLES_H */

