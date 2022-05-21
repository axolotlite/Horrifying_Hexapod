#include <arduino-timer.h>
#include "motion_core.cpp"

static unsigned char motionPercentage;
static unsigned long prevMillis;
static unsigned long currentMillis;  

MOTION legRaise = {
    {initialPoint,initialPoint,initialPoint,initialPoint,initialPoint,initialPoint}, //all legs start at the initial position
    {legOutstretshed,initialPoint,initialPoint,initialPoint,initialPoint,initialPoint}, //all legs end at the initial position except leg 1
    {LINEAR_TRAJECTORY,NONE_TRAJECTORY,NONE_TRAJECTORY,NONE_TRAJECTORY,NONE_TRAJECTORY,NONE_TRAJECTORY},
    10,
    false
};

void setup(){

    Serial.begin(9600);
    //set the PWM driver data
    PWM_Driver.begin();
    PWM_Driver.setOscillatorFrequency(27000000);
    PWM_Driver.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    delay(10);
    leg6.attach();

    //variable definitions
    motionPercentage = 0;
    currentMillis = 0;
    prevMillis = 0;
    startNextMotion(&legRaise);
    Serial.println(legRaise.start[0].x);
}

void loop(){
    if(currentMillis - prevMillis >=20){
        motionProcess(&motionPercentage);
        prevMillis = currentMillis;
    }
    currentMillis = millis();
}
