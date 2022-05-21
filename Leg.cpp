#include <Adafruit_PWMServoDriver.h>
#include "structs.h"
#include <Servo.h>

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  500 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2300 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define MAXROTATION 180 // the maximum rotation applicable for this servo
//#define PWM_2_DEGREE(angle) (USMIN + angle*round((USMAX*1.0f - USMIN)/MAXROTATION*1.0f))  // each step takes one of these bad boys
#define PWM_2_DEGREE(angle) (USMIN + angle*10) 
static Adafruit_PWMServoDriver PWM_Driver = Adafruit_PWMServoDriver(0x40);

class Leg{
    public:
    const static int interval = 10;
    //for initializatoin of the servos.
    POINT position;
    LINK coxa;
    LINK femur;
    LINK tibia;
    //this may be removed later on, mostly because trajectory already exists in motion struct
    //default trajectory type is Linear, so it can intialize at 90 degree angle.
//    TRAJECTORY_TYPE currentTrajectory = LINEAR;
  
    Leg(LINK coxa,LINK femur,LINK tibia){
        this->coxa = coxa;
        this->femur = femur;
        this->tibia = tibia;
    }
    void update(){
      PWM_Driver.writeMicroseconds(this->coxa.pin,PWM_2_DEGREE(coxa.angle));
      PWM_Driver.writeMicroseconds(this->femur.pin,PWM_2_DEGREE(femur.angle));
      PWM_Driver.writeMicroseconds(this->tibia.pin,PWM_2_DEGREE(tibia.angle));
    }
};
class Leg2 : public Leg{
    public:
    Servo coxaServo;
    Servo femurServo;
    // Servo tibiaaServo;
    Leg2(LINK coxa,LINK femur,LINK tibia) : Leg(coxa,femur,tibia){
      Serial.println("is this it?");
    }
    void update(){
        coxaServo.write(coxa.angle);
        femurServo.write(femur.angle);
        PWM_Driver.writeMicroseconds(this->tibia.pin,PWM_2_DEGREE(tibia.angle));
    }
    void attach(){
       coxaServo.attach(coxa.pin);
       femurServo.attach(femur.pin);
   }
    void detach(){
       coxaServo.detach();
       femurServo.detach();
   }
};

//This will need to be modified to fix uneven angles.
static Leg leg1 = Leg(DEFAULT_COXA(13),DEFAULT_FEMUR(14),DEFAULT_TIBIA(15));
static Leg leg2 = Leg(DEFAULT_COXA(10),DEFAULT_FEMUR(11),DEFAULT_TIBIA(12));
static Leg leg3 = Leg(DEFAULT_COXA(7),DEFAULT_FEMUR(8),DEFAULT_TIBIA(9));
static Leg leg4 = Leg(DEFAULT_COXA(4),DEFAULT_FEMUR(5),DEFAULT_TIBIA(6));
static Leg leg5 = Leg(DEFAULT_COXA(1),DEFAULT_FEMUR(2),DEFAULT_TIBIA(3));
static Leg2 leg6 = Leg2(DEFAULT_COXA(6),DEFAULT_FEMUR(5),DEFAULT_TIBIA(0));
// took me long enough to remember up/down casting
//this will allow trajectory calculation in the motion_core but will need recasting for update
 static Leg* legs[6] = {&leg1, &leg2, &leg3, &leg4, &leg5, (Leg *) &leg6};
