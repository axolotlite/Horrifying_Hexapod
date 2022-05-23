#ifndef _MOTION_CORE_
#define _MOTION_CORE_

// #include "constants.h"
#define LIMB_COUNT 6

#define COXA_LENGTH 26
#define COXA_LENGTH_SQUARED 676
#define FEMUR_LENGTH 40
#define FEMUR_LENGTH_SQUARED 1600
#define TIBIA_LENGTH 80
#define TIBIA_LENGTH_SQUARED 6400
#define FEMUR_COXA_SUM 66

// min and max angles of each leg
#define COXA_MIN 25
#define COXA_MAX 165

//the 3d point
typedef struct {
    short int x;
    short int y;
    short int z;
} POINT;


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

static void motionCoreInit();
static void updateAngles();
static bool legInverseKinematics();
static bool linearTrajectory();
static bool arcTrajectory(float arc);
static bool elipticalTrajectory();
static void startNextMotion(MOTION* nextMotion);
static void motionProcess();
//some defaults point, will need to further expand the point system for motion critical points
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

const static MOTION legRaise = {
    {initialPoint,initialPoint,initialPoint,initialPoint,initialPoint,initialPoint}, //all legs start at the initial position
    {legOutstretshed,legOutstretshed,legOutstretshed,legOutstretshed,legOutstretshed,legOutstretshed}, //all legs end at the initial position except leg 1
    {LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY},
    10,
    false
};

//default configs for each link type, will be later removed
#define DEFAULT_COXA(pin) {90,90, COXA_MIN, COXA_MAX,pin}
#define DEFAULT_FEMUR(pin) {90,90, 0, 180,pin}
#define DEFAULT_TIBIA(pin) {90,90, 0, 180,pin}


#endif