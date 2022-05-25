// #include <arduino-timer.h>
#include "Sequence.cpp"
#include "motion_core.cpp"

static unsigned long prevMillis;
static unsigned long currentMillis;  

static short int count = 0;

void setup(){

    Serial.begin(9600);
    motionCoreInit();

    //variable definitions
    currentMillis = 0;
    prevMillis = 0;
    // sequenceSelector(ROTATE_RIGHT_SEQUENCE);
    // startNextMotion(&moveForward0);
    // startNextMotion(&legRaise);
    // Serial.println(legRaise.start[0].x);
    // Serial.println("New data is in.");
    // Serial.print("tanf(90 - COXA_MIN = "); Serial.println(RAD_TO_DEG(tanf(90 - COXA_MIN)));
    // Serial.print("max Z  up: ");Serial.println(maxZUP);
}

void loop(){
    if(currentMillis - prevMillis >= 40){
        // sequenceEngine();
        
        if(checkMotionCompletion()){
            
            switch(count){
                case 0:startNextMotion(&moveForward0);Serial.println("first motion"); break;
                case 1:startNextMotion(&moveForward1);Serial.println("second motion"); break;
                case 2:startNextMotion(&moveForward2);Serial.println("third motion"); break;
                default:{
                    Serial.println("resetting motion");
                    count = -1;
                }
            }
            count++;

        }
        motionProcess();
        prevMillis = currentMillis;
    }
    currentMillis = millis();
}
