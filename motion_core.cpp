#include <math.h>
 #include "Leg.cpp"

#define M_PI                                (3.14159265f)
#define RAD_TO_DEG(rad)                     ((rad) * 180.0f / M_PI)
#define DEG_TO_RAD(deg)                     ((deg) * M_PI / 180.0f)

static MOTION currentMotion = {0};
static unsigned char currentLeg = 0;
static bool motionComplete = true;
//can be modified later
static unsigned char increment = 1;

static bool legInverseKinematics(){
    float coxa_zero_rotate_deg = DEG_TO_RAD(legs[currentLeg]->coxa.zero_rotate);
    float femur_zero_rotate_deg = DEG_TO_RAD(legs[currentLeg]->femur.zero_rotate);
    float tibia_zero_rotate_deg = DEG_TO_RAD(legs[currentLeg]->tibia.zero_rotate);
//    Serial.println(coxa_zero_rotate_deg);
    float x1 = legs[currentLeg]->position.x * cosf(coxa_zero_rotate_deg) + legs[currentLeg]->position.z * sinf(coxa_zero_rotate_deg);
    float y1 = legs[currentLeg]->position.y;
    float z1 = -legs[currentLeg]->position.x * sinf(coxa_zero_rotate_deg) + legs[currentLeg]->position.z * cosf(coxa_zero_rotate_deg);
    Serial.print("x: ");Serial.println(x1);
    Serial.print("y: ");Serial.println(y1);
    Serial.print("z: ");Serial.println(z1);
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

    float distance = sqrt( x1 * x1 + y1 * y1);
    if(distance < FEMUR_COXA_SUM ){
        return false;
    }
    
    float alpha = acosf((FEMUR_LENGTH_SQUARED + distance * distance - TIBIA_LENGTH_SQUARED) / (2.0f * FEMUR_LENGTH * distance));
    float gamma = acosf(( TIBIA_LENGTH_SQUARED + FEMUR_LENGTH_SQUARED - distance * distance)/(2.0f * FEMUR_LENGTH * TIBIA_LENGTH));

    legs[currentLeg]->femur.angle = legs[currentLeg]->femur.zero_rotate - RAD_TO_DEG(alpha) - RAD_TO_DEG(fi);

    //***********IMPORTANT NOTE*************//
    //not subtracting the tibia zero rotation makes this equation work...
    //TODO actually test that shit
    legs[currentLeg]->tibia.angle = RAD_TO_DEG(gamma); //- legs[currentLeg]->tibia.zero_rotate;

    legs[currentLeg]->coxa.angle =round(fabs(legs[currentLeg]->coxa.angle));
    legs[currentLeg]->femur.angle = round(legs[currentLeg]->femur.angle);
    legs[currentLeg]-> tibia.angle = round(legs[currentLeg]->tibia.angle);
//    Serial.println("First");
//    Serial.print("coxa = ");Serial.print(legs[currentLeg]->coxa.angle);
//    Serial.print(", femur = ");Serial.print(legs[currentLeg]->femur.angle);
//    Serial.print(", tibia = ");Serial.println(legs[currentLeg]->tibia.angle);
    
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
    Serial.print("coxa = ");Serial.print(legs[currentLeg]->coxa.angle);
    Serial.print(", femur = ");Serial.print(legs[currentLeg]->femur.angle);
    Serial.print(", tibia = ");Serial.println(legs[currentLeg]->tibia.angle);
    return true;
}

static bool linearTrajectory(float motionPercentage) {
//     if(currentMotion.isDone)
//     return true;
// //    float relativeMotionTime = motion.motionTime;
//     if(motionTime >= 0.99){
//       currentMotion.isDone = true;
//       motionTime = 0;
//       return true;  
//     }

    // motionTime += increment;
    float x0 = currentMotion.start[currentLeg].x;
    float y0 = currentMotion.start[currentLeg].y;
    float z0 = currentMotion.start[currentLeg].z;
    float x1 = currentMotion.destination[currentLeg].x;
    float y1 = currentMotion.destination[currentLeg].y;
    float z1 = currentMotion.destination[currentLeg].z;
    
    legs[currentLeg]->position.x = x0 + motionPercentage * (x1 - x0);
    legs[currentLeg]->position.y = y0 + motionPercentage * (y1 - y0);
    legs[currentLeg]->position.z = z0 + motionPercentage * (z1 - z0);
    Serial.println("LinearMotion");
    Serial.print("x = ");Serial.print(legs[currentLeg]->position.x);
    Serial.print(", y = ");Serial.print(legs[currentLeg]->position.y);
    Serial.print(", z = ");Serial.println(legs[currentLeg]->position.z);

    //    std::cout << "x = " << legs[currentLeg]->position.x << ", y = " << legs[currentLeg]->position.y << " , z = " << legs[currentLeg]->position.z << '\n';
    
    return false;
}
//there should be handling of arc type
static bool arcTrajectory(float motionPercentage,float arc) {
//     if( currentMotion.isDone)
//     return true;
// //    float relativeMotionTime = motion.motionTime;
//     if(motionTime >= 0.99){
//       currentMotion.isDone = true;
//       motionTime = 0;
//       return true;  
//     }

    // motionTime += increment;
    float x0 = currentMotion.start[currentLeg].x;
    float y0 = currentMotion.start[currentLeg].y;
    float z0 = currentMotion.start[currentLeg].z;
    float x1 = currentMotion.destination[currentLeg].x;
    float y1 = currentMotion.destination[currentLeg].y;
    float z1 = currentMotion.destination[currentLeg].z;
    
    float r = sqrt(x0* x0 + z0 * z0);
//    Serial.println("r = ");Serial.println(
    float atan0 = RAD_TO_DEG(atan2(x0, z0));
    float atan1 = RAD_TO_DEG(atan2(x1, z1));
    float t_mapped_rad = DEG_TO_RAD(motionPercentage * (atan1 - atan0) + atan0);
    
    legs[currentLeg]->position.x = r * sin(t_mapped_rad); // Circle Y
//    legs[currentLeg]->position.y = y0 * sin(DEG_TO_RAD(motion->motionTime)) + y0;
    legs[currentLeg]->position.y = y0 + arc * r * sin(DEG_TO_RAD(180 * motionPercentage ));
    legs[currentLeg]->position.z = r * cos(t_mapped_rad); // Circle X
//    legs[currentLeg]->position.x = x0 + motion->motionTime * (x1 - x0);
//    legs[currentLeg]->position.y = y0 + motion->motionTime * (y1 - y0);
//    legs[currentLeg]->position.z = z0 + motion->motionTime * (z1 - z0);

//    Serial.print("x = ");Serial.print(legs[currentLeg]->position.x);
//    Serial.print(", y = ");Serial.print(legs[currentLeg]->position.y);
//    Serial.print(", z = ");Serial.println(legs[currentLeg]->position.z);

    //    std::cout << "x = " << legs[currentLeg]->position.x << ", y = " << legs[currentLeg]->position.y << " , z = " << legs[currentLeg]->position.z << '\n';
    
    return false;
}

static bool elipticalTrajectory(float motionPercentage) {
//    float relativeMotionTime = motion.motionTime;
//huge possibilit of depricating this
    // if(motionPercentage >= 100){
    //   currentMotion.isDone = true;
    //   motionPercentage = 0;
    //   return true;  
    // }
    // motionTime += increment;

    float x0 = currentMotion.start[currentLeg].x;
    float y0 = currentMotion.start[currentLeg].y;
    float z0 = currentMotion.start[currentLeg].z;
    float x1 = currentMotion.destination[currentLeg].x;
    float y1 = currentMotion.destination[currentLeg].y;
    float z1 = currentMotion.destination[currentLeg].z;

    float a = (z1 - z0) / 2.0f;
    float b = (x1 - x0);
    float c = (y1 - y0);

    legs[currentLeg]->position.x = b * sin(DEG_TO_RAD(180.0f - 90 * motionPercentage )) + x0; // circle Y
    legs[currentLeg]->position.y = c * sin(DEG_TO_RAD(90 * motionPercentage )) + y0;
    
    legs[currentLeg]->position.z = a * cos(DEG_TO_RAD(180.0f - 90 * motionPercentage )) + z0 + a;

//    Serial.print("x = ");Serial.print(legs[currentLeg]->position.x);
//    Serial.print(", y = ");Serial.print(legs[currentLeg]->position.y);
//    Serial.print(", z = ");Serial.println(legs[currentLeg]->position.z);
//    
//    Serial.println(motion->motionTime);
    return false;
}


static void startNextMotion(MOTION* nextMotion){
    currentMotion = *nextMotion;
//    Serial.println(nextMotion->start[0].y);
//    Serial.println("Current motion y: ");
//    Serial.println(currentMotion.start[0].y);
}
static void motionProcess(unsigned char* motionPercentage){
    //check if motion is done and percentage should be reset.
    //sensor data goes here

    //checks the motion to see if its done or not.
    if(!currentMotion.isDone){
        Serial.println("Current motion y: ");
        Serial.println(currentMotion.start[0].y);
        // float motionTime = 0;
        float relativeMotionPercentage = *motionPercentage / 100.0f;
        //calculate the trajectory for each leg
        //unsure if i should put the inverse kinematics over here aswell.
        for(currentLeg = 0; currentLeg < 6; currentLeg++){
            // if(legs[0]->currentTrajectory){}
            //calls function according to trajectory type
//            Serial.println
            switch(currentMotion.trajectory[currentLeg]){
                case NONE_TRAJECTORY:continue;
                //get leg from one point to the next in a straight line, works on xz axis
                case LINEAR_TRAJECTORY:linearTrajectory(relativeMotionPercentage);break;
                //rotate according to the coxa, works without changing the y-axis
                case LINEAR_ARC_TRAJECTORY:arcTrajectory(relativeMotionPercentage, 0);break;
                //rotate according to the coxa, while changing/raising the y-axis
                case ARC_TRAJECTORY:arcTrajectory(relativeMotionPercentage, 0.25);break;
                //moves leg from one point to the next in an elipse, works on xz axis
                case ELIPTICAL_TRAJECTORY:elipticalTrajectory(relativeMotionPercentage);
                //some bug is causing it to default.
                default:break;
            }
            if(!legInverseKinematics()){
                Serial.println("[Inverse Kinematics Error]Point out of reach, failing to calculate angles");
            }
            //this will call the update function containing the two older servos
            currentLeg == 6 ? ((Leg2 *) legs[currentLeg])->update() : legs[currentLeg]->update();
            *motionPercentage += increment;
            //if the motion is greater than 100 then it's done
            if(*motionPercentage > 100){
                Serial.print("percentage = ");Serial.println(*motionPercentage);
                currentMotion.isDone = true;
                *motionPercentage = 0;
            }
        }

    }

}
