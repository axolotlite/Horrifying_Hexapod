// #include <arduino-timer.h>
#include "Sequence.cpp"
#include "motion_core.cpp"

static unsigned long prevMillis;
static unsigned long currentMillis;  

static short int count = 0;
static unsigned char flag1 = 12;
static unsigned char flag2;
static unsigned char flag3;
int offset = 10;
void setup(){
    pinMode(flag1,INPUT);
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
    // moveForward();
    flag2 = digitalRead(flag1);
    if(flag2 && checkMotionCompletion())
        setNextMotion(&legRaise);
    else if(!flag2 && checkMotionCompletion())
        setNextMotion(&legReturn)
    if(currentMillis - prevMillis >= 40){
        // sequenceEngine();
        
        // if(checkMotionCompletion()){
            
        //     switch(count){
        //         case 0:startNextMotion(&moveForward0);Serial.println("first motion"); break;
        //         case 1:startNextMotion(&moveForward1);Serial.println("second motion"); break;
        //         case 2:startNextMotion(&moveForward2);Serial.println("third motion"); break;
        //         default:{
        //             Serial.println("resetting motion");
        //             count = -1;
        //         }
        //     }
        //     count++;

        // }
        updateAngles();
        motionProcess();
        prevMillis = currentMillis;
    }
    currentMillis = millis();
}

void moveForward(){
    PWM_Driver.writeMicroseconds(legs[0].coxa.pin,PWM_2_DEGREE(103));
    PWM_Driver.writeMicroseconds(legs[1].coxa.pin,PWM_2_DEGREE(83));


    delay(2000);
    PWM_Driver.writeMicroseconds(legs[4].coxa.pin,PWM_2_DEGREE(83));
    // leg6Coxa.write(legs[5].coxa.angle - offset);
    if(updateAnglesFlag){
        Serial.print("leg1: ");Serial.print(legs[0].coxa.angle);Serial.print(", ");Serial.print(legs[0].femur.angle);Serial.print(", ");Serial.println(legs[0].tibia.angle);
        Serial.print("leg2: ");Serial.print(legs[1].coxa.angle);Serial.print(", ");Serial.print(legs[1].femur.angle);Serial.print(", ");Serial.println(legs[1].tibia.angle);
        Serial.print("leg3: ");Serial.print(legs[2].coxa.angle);Serial.print(", ");Serial.print(legs[2].femur.angle);Serial.print(", ");Serial.println(legs[2].tibia.angle);
        Serial.print("leg4: ");Serial.print(legs[3].coxa.angle);Serial.print(", ");Serial.print(legs[3].femur.angle);Serial.print(", ");Serial.println(legs[3].tibia.angle);
        Serial.print("leg5: ");Serial.print(legs[4].coxa.angle);Serial.print(", ");Serial.print(legs[4].femur.angle);Serial.print(", ");Serial.println(legs[4].tibia.angle);
        Serial.print("leg6: ");Serial.print(legs[5].coxa.angle);Serial.print(", ");Serial.print(legs[5].femur.angle);Serial.print(", ");Serial.println(legs[5].tibia.angle);
    }
    delay(2000);
    resetAngles();
    updateAngles();
    delay(2000);
}
