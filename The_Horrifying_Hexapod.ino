#include <arduino-timer.h>
#include "motion_core.cpp"

static unsigned long prevMillis;
static unsigned long currentMillis;  



void setup(){

    Serial.begin(9600);
    motionCoreInit();

    //variable definitions
    currentMillis = 0;
    prevMillis = 0;
    startNextMotion(&legRaise);
    Serial.println(legRaise.start[0].x);
}

void loop(){
    if(currentMillis - prevMillis >=20){
        motionProcess();
        prevMillis = currentMillis;
    }
    currentMillis = millis();
}
