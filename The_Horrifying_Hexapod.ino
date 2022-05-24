// #include <arduino-timer.h>
#include "motion_core.cpp"

static unsigned long prevMillis;
static unsigned long currentMillis;  



void setup(){

    Serial.begin(9600);
    motionCoreInit();

    //variable definitions
    currentMillis = 0;
    prevMillis = 0;
    // startNextMotion(&legRaise);
    startNextMotion(&debugMotion);
    // Serial.println(legRaise.start[0].x);
    // Serial.println("New data is in.");
    // Serial.print("tanf(90 - COXA_MIN = "); Serial.println(RAD_TO_DEG(tanf(90 - COXA_MIN)));
    // Serial.print("max Z  up: ");Serial.println(maxZUP);
}

void loop(){
    if(currentMillis - prevMillis >=20){
        motionProcess();
        prevMillis = currentMillis;
    }
    currentMillis = millis();
}
