#include "constants.h"
#ifndef _STRUCTS_
#define _STRUCTS_

//the 3d point
typedef struct {
    short int x;
    short int y;
    short int z;
} POINT;
//the rotation point of the servo and connection length to the next point
typedef struct{
    float angle;
    //uneeded
    // unsigned char length; //length of the link
    unsigned char zero_rotate; //starting or default angle of the motor
    unsigned char min_angle; //maximum angle
    unsigned char max_angle; //minimum angle
    unsigned char pin;
}LINK;

//deprecated, currently used for debugging

typedef struct {
   POINT position;
   LINK coxa;
   LINK femur;
   LINK tibia;
}LIMB;

typedef enum{
    NONE_TRAJECTORY,
    LINEAR_TRAJECTORY,
    LINEAR_ARC_TRAJECTORY,
    ARC_TRAJECTORY,
    ELIPTICAL_TRAJECTORY
}TRAJECTORY_TYPE;
//motion information for trajectory calculation
typedef struct{
    POINT start[6];
    POINT destination[6];
    TRAJECTORY_TYPE trajectory[6];
    // float motionTime;
    unsigned char motionSpeed;
    bool isDone;
}MOTION;

typedef struct {
    int speed;
    int curvature;
    int distance;
} MOTION_CONFIG;

typedef struct {
    bool loopable;
    unsigned char motionCount;
    MOTION* motions[10];
}SEQUENCE;

const static POINT initialPoint = {
    COXA_LENGTH + FEMUR_LENGTH,
    -TIBIA_LENGTH,
    0
};
const static POINT idlePoint = {
    COXA_LENGTH + FEMUR_LENGTH,
    -TIBIA_LENGTH,
    -20
};
const static POINT legOutstretshed = {
    COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH,
    0,
    0
};
// const static POINT rotateRight = {
//   0,
//     -TIBIA_LENGTH,
//     COXA_LENGTH + FEMUR_LENGTH
//   };

#endif
