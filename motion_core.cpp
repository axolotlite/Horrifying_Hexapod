// #include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include "debug.h"
#include "motion_core.h"

#include "servo_driver_configs.h"
//  #include "Leg.cpp"



typedef struct{
    float angle;
    //uneeded
    // unsigned char length; //length of the link
    unsigned char zero_rotate; //starting or default angle of the motor
    unsigned char min_angle; //maximum angle
    unsigned char max_angle; //minimum angle
    unsigned char pin;
}LINK;

typedef struct {
   POINT point;
   LINK coxa;
   LINK femur;
   LINK tibia;
}LIMB;

static MOTION currentMotion = {0};
static unsigned char currentLeg = 0;
static bool motionComplete = false;
//can be modified later
static float motionPercentage = 0;
static unsigned char increment = 1;

//The servo driver
static Adafruit_PWMServoDriver PWM_Driver = Adafruit_PWMServoDriver(0x40);

//The legs
static LIMB leg1 = {initialPoint,DEFAULT_COXA(13),DEFAULT_FEMUR(14),DEFAULT_TIBIA(15)};
static LIMB leg2 = {initialPoint,DEFAULT_COXA(10),DEFAULT_FEMUR(11),DEFAULT_TIBIA(12)};
static LIMB leg3 = {initialPoint,DEFAULT_COXA(7),DEFAULT_FEMUR(8),DEFAULT_TIBIA(9)};
static LIMB leg4 = {initialPoint,DEFAULT_COXA(4),DEFAULT_FEMUR(5),DEFAULT_TIBIA(6)};
static LIMB leg5 = {initialPoint,DEFAULT_COXA(1),DEFAULT_FEMUR(2),DEFAULT_TIBIA(3)};
static LIMB leg6 = {initialPoint,DEFAULT_COXA(A1),DEFAULT_FEMUR(A0),DEFAULT_TIBIA(0)};
static LIMB *legs[6] = {&leg1, &leg2, &leg3, &leg4, &leg5, &leg6};
static Servo leg6Coxa;
static Servo leg6Femur;

static void motionCoreInit(){
    leg6Coxa.attach(leg6.coxa.pin);
    leg6Femur.attach(leg6.femur.pin);
        //set the PWM driver data
    PWM_Driver.begin();
    PWM_Driver.setOscillatorFrequency(27000000);
    PWM_Driver.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    updateAngles();
    // delay(10);
    // leg6.attach();
}

static void updateAngles(){
    
    PWM_Driver.writeMicroseconds(leg1.coxa.pin,PWM_2_DEGREE(leg1.coxa.angle));
    PWM_Driver.writeMicroseconds(leg1.femur.pin,PWM_2_DEGREE(leg1.femur.angle));
    PWM_Driver.writeMicroseconds(leg1.tibia.pin,PWM_2_DEGREE(leg1.tibia.angle));

    PWM_Driver.writeMicroseconds(leg2.coxa.pin,PWM_2_DEGREE(leg2.coxa.angle));
    PWM_Driver.writeMicroseconds(leg2.femur.pin,PWM_2_DEGREE(leg2.femur.angle));
    PWM_Driver.writeMicroseconds(leg2.tibia.pin,PWM_2_DEGREE(leg2.tibia.angle));

    PWM_Driver.writeMicroseconds(leg3.coxa.pin,PWM_2_DEGREE(leg3.coxa.angle));
    PWM_Driver.writeMicroseconds(leg3.femur.pin,PWM_2_DEGREE(leg3.femur.angle));
    PWM_Driver.writeMicroseconds(leg3.tibia.pin,PWM_2_DEGREE(leg3.tibia.angle));

    PWM_Driver.writeMicroseconds(leg4.coxa.pin,PWM_2_DEGREE(leg4.coxa.angle));
    PWM_Driver.writeMicroseconds(leg4.femur.pin,PWM_2_DEGREE(leg4.femur.angle));
    PWM_Driver.writeMicroseconds(leg4.tibia.pin,PWM_2_DEGREE(leg4.tibia.angle));

    PWM_Driver.writeMicroseconds(leg5.coxa.pin,PWM_2_DEGREE(leg5.coxa.angle));
    PWM_Driver.writeMicroseconds(leg5.femur.pin,PWM_2_DEGREE(leg5.femur.angle));
    PWM_Driver.writeMicroseconds(leg5.tibia.pin,PWM_2_DEGREE(leg5.tibia.angle));

    leg6Coxa.write(leg6.coxa.angle);
    leg6Femur.write(leg6.femur.angle);
    PWM_Driver.writeMicroseconds(leg6.tibia.pin,PWM_2_DEGREE(leg6.tibia.angle));
    if(updateAnglesFlag){
        Serial.print("leg1: ");Serial.print(leg1.coxa.angle);Serial.print(", ");Serial.print(leg1.femur.angle);Serial.print(", ");Serial.println(leg1.tibia.angle);
        Serial.print("leg2: ");Serial.print(leg2.coxa.angle);Serial.print(", ");Serial.print(leg2.femur.angle);Serial.print(", ");Serial.println(leg2.tibia.angle);
        Serial.print("leg3: ");Serial.print(leg3.coxa.angle);Serial.print(", ");Serial.print(leg3.femur.angle);Serial.print(", ");Serial.println(leg3.tibia.angle);
        Serial.print("leg4: ");Serial.print(leg4.coxa.angle);Serial.print(", ");Serial.print(leg4.femur.angle);Serial.print(", ");Serial.println(leg4.tibia.angle);
        Serial.print("leg5: ");Serial.print(leg5.coxa.angle);Serial.print(", ");Serial.print(leg5.femur.angle);Serial.print(", ");Serial.println(leg5.tibia.angle);
        Serial.print("leg6: ");Serial.print(leg6.coxa.angle);Serial.print(", ");Serial.print(leg6.femur.angle);Serial.print(", ");Serial.println(leg6.tibia.angle);
    }
}

static bool legInverseKinematics(){
    float coxa_zero_rotate_deg = DEG_TO_RAD(legs[currentLeg]->coxa.zero_rotate);
    float femur_zero_rotate_deg = DEG_TO_RAD(legs[currentLeg]->femur.zero_rotate);
    float tibia_zero_rotate_deg = DEG_TO_RAD(legs[currentLeg]->tibia.zero_rotate);
    float x1 = legs[currentLeg]->point.x * cosf(coxa_zero_rotate_deg) + legs[currentLeg]->point.z * sinf(coxa_zero_rotate_deg);
    float y1 = legs[currentLeg]->point.y;
    float z1 = -legs[currentLeg]->point.x * sinf(coxa_zero_rotate_deg) + legs[currentLeg]->point.z * cosf(coxa_zero_rotate_deg);
    if(legInverseKinematicsSecondDebugFlag){
        Serial.println("[Leg IK Data]Leg positions");
        Serial.print("x = ");Serial.println(legs[currentLeg]->point.x);
        Serial.print("y = ");Serial.println(legs[currentLeg]->point.y);
        Serial.print("z = ");Serial.println(legs[currentLeg]->point.z);
        Serial.println("[Leg IK Data]End");
    }
    //may return a negative value.
    float new_coxa_angle_rad = atan2f(z1,x1);
//    Serial.print("new coxa angle");
//    Serial.println(new_coxa_angle_rad);
    legs[currentLeg]->coxa.angle = RAD_TO_DEG(new_coxa_angle_rad);
    x1 = x1 * cosf(new_coxa_angle_rad) + z1 * sinf(new_coxa_angle_rad);
    // std::cout << "x1 = " << x1 << '\n';
    x1 = x1 - COXA_LENGTH;//legs[currentLeg]->coxa.length;
    // std::cout << "x1 = " << x1 << '\n';

    float fi = atan2f(y1,x1);
    //need to check if this shit fucks any other shit up, nigger.
    float distance = sqrt( x1 * x1 + y1 * y1);
    if(distance > FEMUR_TIBIA_SUM){
        // Serial.println(FEMUR_COXA_SUM);
        // Serial.print("point: ");Serial.print(x1);Serial.print(", ");Serial.print(y1);Serial.print(", ");Serial.print(z1);Serial.print(", ");
        return false;
    }
    
    float alpha = acosf((FEMUR_LENGTH_SQUARED + distance * distance - TIBIA_LENGTH_SQUARED) / (2.0f * FEMUR_LENGTH * distance));
    float gamma = acosf((TIBIA_LENGTH_SQUARED + FEMUR_LENGTH_SQUARED - distance * distance)/(2.0f * FEMUR_LENGTH * TIBIA_LENGTH));

    legs[currentLeg]->femur.angle = legs[currentLeg]->femur.zero_rotate - RAD_TO_DEG(alpha) - RAD_TO_DEG(fi);

    //****IMPORTANT NOTE****//
    //not subtracting the tibia zero rotation makes this equation work...
    //TODO actually test that shit
    legs[currentLeg]->tibia.angle = RAD_TO_DEG(gamma); //- legs[currentLeg]->tibia.zero_rotate;

    legs[currentLeg]->coxa.angle =round(fabs(legs[currentLeg]->coxa.angle));
    legs[currentLeg]->femur.angle = round(legs[currentLeg]->femur.angle);
    legs[currentLeg]->tibia.angle = round(legs[currentLeg]->tibia.angle);
    if(legInverseKinematicsSecondDebugFlag){
        Serial.println("[Leg IK Data]Angle pre-safety measure");
        Serial.print("coxa = ");Serial.println(legs[currentLeg]->coxa.angle);
        Serial.print(", femur = ");Serial.println(legs[currentLeg]->femur.angle);
        Serial.print(", tibia = ");Serial.println(legs[currentLeg]->tibia.angle);
        Serial.println("[Leg IK Data]End");
   }
    
    //make sure they don't exceed the maximum nor minimum angle
    if (legs[currentLeg]->coxa.angle < legs[currentLeg]->coxa.min_angle) {
        legs[currentLeg]->coxa.angle = legs[currentLeg]->coxa.min_angle;
    }
    if (legs[currentLeg]->coxa.angle > legs[currentLeg]->coxa.max_angle) {
        legs[currentLeg]->coxa.angle  = legs[currentLeg]->coxa.max_angle;
    }
    if (legs[currentLeg]->femur.angle < legs[currentLeg]->femur.min_angle) {
        legs[currentLeg]->femur.angle = legs[currentLeg]->femur.min_angle;
    }
    if (legs[currentLeg]->femur.angle > legs[currentLeg]->femur.max_angle) {
        legs[currentLeg]->femur.angle = legs[currentLeg]->femur.max_angle;
    }
    if (legs[currentLeg]->tibia.angle < legs[currentLeg]->tibia.min_angle) {
        legs[currentLeg]->tibia.angle = legs[currentLeg]->tibia.min_angle;
    }
    if (legs[currentLeg]->tibia.angle > legs[currentLeg]->tibia.max_angle) {
        legs[currentLeg]->tibia.angle = legs[currentLeg]->tibia.max_angle;
    }
//    Serial.println("angles");
    
    if(legInverseKinematicsFirstDebugFlag){
        Serial.println("[Leg IK Data]Begin");
        Serial.print("target x: ");Serial.println(x1);
        Serial.print("target y: ");Serial.println(y1);
        Serial.print("target z: ");Serial.println(z1);
        Serial.println("[Leg IK Data]Angle post-safety measure");
        Serial.print("coxa = ");Serial.println(legs[currentLeg]->coxa.angle);
        Serial.print("femur = ");Serial.println(legs[currentLeg]->femur.angle);
        Serial.print("tibia = ");Serial.println(legs[currentLeg]->tibia.angle);
        Serial.println("[Leg IK Data]End");
    }

    return true;
}

static bool linearTrajectory() {
    short int x0 = currentMotion.start[currentLeg].x;
    short int y0 = currentMotion.start[currentLeg].y;
    short int z0 = currentMotion.start[currentLeg].z;
    short int x1 = currentMotion.destination[currentLeg].x;
    short int y1 = currentMotion.destination[currentLeg].y;
    short int z1 = currentMotion.destination[currentLeg].z;

    legs[currentLeg]->point.x = x0 + motionPercentage * (x1 - x0);
    legs[currentLeg]->point.y = y0 + motionPercentage * (y1 - y0);
    legs[currentLeg]->point.z = z0 + motionPercentage * (z1 - z0);
    
    if(linearTrajectoryFlag){
        Serial.println("[Linear Motion info]Begin");

        Serial.print("x = ");Serial.println(legs[currentLeg]->point.x);
        Serial.print("y = ");Serial.println(legs[currentLeg]->point.y);
        Serial.print("z = ");Serial.println(legs[currentLeg]->point.z);

        Serial.println("[Linear Motion info]End");
    }

    
    
    

    //    std::cout << "x = " << legs[currentLeg]->point.x << ", y = " << legs[currentLeg]->point.y << " , z = " << legs[currentLeg]->point.z << '\n';
    
    return false;
}
//there should be handling of arc type
static bool arcTrajectory(float arc) {
    float x0 = currentMotion.start[currentLeg].x;
    float y0 = currentMotion.start[currentLeg].y;
    float z0 = currentMotion.start[currentLeg].z;
    float x1 = currentMotion.destination[currentLeg].x;
    float y1 = currentMotion.destination[currentLeg].y;
    float z1 = currentMotion.destination[currentLeg].z;
    
    float r = sqrt(x0* x0 + z0 * z0);
    float atan0 = RAD_TO_DEG(atan2(x0, z0));
    float atan1 = RAD_TO_DEG(atan2(x1, z1));
    float t_mapped_rad = DEG_TO_RAD(motionPercentage * (atan1 - atan0) + atan0);
    
    legs[currentLeg]->point.x = r * sin(t_mapped_rad); 
    legs[currentLeg]->point.y = y0 + arc * r * sin(DEG_TO_RAD(180 * motionPercentage ));
    legs[currentLeg]->point.z = r * cos(t_mapped_rad);

//    legs[currentLeg]->point.x = x0 + motion->motionTime * (x1 - x0);
//    legs[currentLeg]->point.y = y0 + motion->motionTime * (y1 - y0);
//    legs[currentLeg]->point.z = z0 + motion->motionTime * (z1 - z0);

    if(arcTrajectoryFlag){
        Serial.println("[Arc Motion info]Begin");
        Serial.print("x = ");Serial.println(legs[currentLeg]->point.x);
        Serial.print("y = ");Serial.println(legs[currentLeg]->point.y);
        Serial.print("z = ");Serial.println(legs[currentLeg]->point.z);
        Serial.println("[Arc Motion info]End");
    }

    //    std::cout << "x = " << legs[currentLeg]->point.x << ", y = " << legs[currentLeg]->point.y << " , z = " << legs[currentLeg]->point.z << '\n';
    
    return false;
}

static bool elipticalTrajectory() {

    float x0 = currentMotion.start[currentLeg].x;
    float y0 = currentMotion.start[currentLeg].y;
    float z0 = currentMotion.start[currentLeg].z;
    float x1 = currentMotion.destination[currentLeg].x;
    float y1 = currentMotion.destination[currentLeg].y;
    float z1 = currentMotion.destination[currentLeg].z;

    float a = (z1 - z0) / 2.0f;
    float b = (x1 - x0);
    float c = (y1 - y0);

    legs[currentLeg]->point.x = b * sin(DEG_TO_RAD(180.0f - 90 * motionPercentage )) + x0;
    legs[currentLeg]->point.y = c * sin(DEG_TO_RAD(90 * motionPercentage )) + y0;
    legs[currentLeg]->point.z = a * cos(DEG_TO_RAD(180.0f - 90 * motionPercentage )) + z0 + a;

    if(elipticalTrajectoryFlag){
        Serial.println("[Eliptical Motion info]Begin");
        Serial.print("x = ");Serial.println(legs[currentLeg]->point.x);
        Serial.print("y = ");Serial.println(legs[currentLeg]->point.y);
        Serial.print("z = ");Serial.println(legs[currentLeg]->point.z);
        Serial.println("[Eliptical Motion info]End");
    }
//    
//    Serial.println(motion->motionTime);
    return false;
}

static bool checkMotionCompletion(){
    return motionComplete;
}
static void startNextMotion(MOTION* nextMotion){
    currentMotion = *nextMotion;
//    Serial.println(nextMotion->start[0].y);
//    Serial.println("Current motion y: ");
//    Serial.println(currentMotion.start[0].y);
}
static void motionProcess(){
    //check if motion is done and percentage should be reset.
    //sensor data goes here

    //checks the motion to see if its done or not.
    if(!motionComplete){
//      This can't ever be here
        motionPercentage += 0.05;

        for(currentLeg = 0; currentLeg < 6; currentLeg++){
            switch(currentMotion.trajectory[currentLeg]){
                case NONE_TRAJECTORY:continue;
                //get leg from one point to the next in a straight line, works on xz axis
                case LINEAR_TRAJECTORY:if(linearTrajectory())continue; break;
                //rotate according to the coxa, works without changing the y-axis
                case LINEAR_ARC_TRAJECTORY:if(arcTrajectory(0))continue ; break;
                //rotate according to the coxa, while changing/raising the y-axis
                case ARC_TRAJECTORY:if(arcTrajectory(0.25))continue; break;
                //moves leg from one point to the next in an elipse, works on xz axis
                case ELIPTICAL_TRAJECTORY:if(elipticalTrajectory()) continue ; break;
                //some bug is causing it to default.
                default:break;
            }
            if(!legInverseKinematics())
                Serial.println("[Inverse Kinematics Error]Point out of reach, failing to calculate angles");
            //this is proof that i'm mentally retarded
            // motionPercentage += 0.01;
            //if the motion is greater than 100 then it's done
            
        }
        updateAngles();
        if(motionPercentage >= 1){
                Serial.print("percentage = ");Serial.println(motionPercentage);
                //remove the boolean from the motion and make it a local variable
                motionComplete = true;

                motionPercentage = 0;
            }
    }

}

