#ifndef _MOTION_CORE_
#define _MOTION_CORE_
#include <math.h>
// #include "constants.h"
#define LIMB_COUNT 6

#define COXA_LENGTH (unsigned char) 30
#define COXA_LENGTH_SQUARED (short int) COXA_LENGTH*COXA_LENGTH
#define FEMUR_LENGTH (unsigned char) 45
#define FEMUR_LENGTH_SQUARED (short int) FEMUR_LENGTH*FEMUR_LENGTH
#define TIBIA_LENGTH (unsigned char) 80
#define TIBIA_LENGTH_SQUARED (short int) TIBIA_LENGTH*TIBIA_LENGTH

#define FEMUR_COXA_SUM (unsigned char) (COXA_LENGTH+FEMUR_LENGTH)
#define FEMUR_TIBIA_SUM (unsigned char) (FEMUR_LENGTH+TIBIA_LENGTH)
#define FEMUR_COXA_DIFF (unsigned char) (FEMUR_LENGTH-COXA_LENGTH)

#define ZERO_ANGLE 90
//these are the equations responsible for determining the points of 45 degrees, but it doesn't work. i don't know why and i don't care.
// #define INITIAL_X (unsigned char) (COXA_LENGTH + FEMUR_LENGTH * cos( 0.5 * 90))
// #define INITIAL_Y (unsigned char) (TIBIA_LENGTH - FEMUR_LENGTH * sin( 0.5 * 90))
// #define INITIAL_Z 0
#define INITIAL_X (unsigned char) 54
#define INITIAL_Y (unsigned char) -52
#define INITIAL_Z (unsigned char) 0

// min and max angles of each leg
#define COXA_MIN (unsigned char) 25
#define COXA_MAX (short int) 165

#define FEMUR_MIN (unsigned char) 25
#define FEMUR_MAX (short int) 165

#define TIBIA_MIN (unsigned char) 25
#define TIBIA_MAX (short int) 165

#define M_PI                                (3.14159265f)
#define RAD_TO_DEG(rad)                     ((rad) * 180.0f / M_PI)
#define DEG_TO_RAD(deg)                     ((deg) * M_PI / 180.0f)

//need further testing because tan returns different point from the calculated z
// #define maxZUP (short int )round(FEMUR_COXA_SUM * RAD_TO_DEG(tanf(90 - COXA_MIN)))
#define maxZUP (short int) 142
#define maxZDown -maxZUP
#define SAFETY_OFFSET 10


//the 3d point
//diagram done
typedef struct {
    short int x;
    short int y;
    short int z;
} POINT;

//diagram done
typedef enum{
    NONE_TRAJECTORY,
    LINEAR_TRAJECTORY,
    LINEAR_ARC_TRAJECTORY,
    ARC_TRAJECTORY,
    ELIPTICAL_TRAJECTORY
}TRAJECTORY_TYPE;
//diagram done
//motion information for trajectory calculation
typedef struct{
    POINT start[6];
    POINT destination[6];
    TRAJECTORY_TYPE trajectory[6];
    //depreicated variables for memory conservation
    // float motionTime;
    // unsigned char motionSpeed;
    // bool isDone;
}MOTION;


static void motionCoreInit();
static void updateAngles();
static bool legInverseKinematics();
static bool linearTrajectory();
static bool arcTrajectory(float arc);
static bool elipticalTrajectory();
static void startNextMotion(MOTION* nextMotion);
static void motionProcess();
static bool checkMotionCompletion();
//some defaults point, will need to further expand the point system for motion critical points

//should initialize body at 90 degree angles
const static POINT initialPoint = {
    FEMUR_COXA_SUM,
    -TIBIA_LENGTH,
    0
};
const static POINT neoInitial = {
    54,
    -52,
    0
};
const static POINT forwardPoint = {
    33,
    -TIBIA_LENGTH,
    -57
};
const static POINT backwardPoint = {
    33,
    -TIBIA_LENGTH,
    57
};
const static POINT idlePoint = {
    FEMUR_COXA_SUM,
    -TIBIA_LENGTH,
    -20
};
const static POINT maxUp = {
    FEMUR_COXA_SUM,
    -TIBIA_LENGTH,
    maxZUP
};
const static POINT halfUP = {
    FEMUR_COXA_SUM,
    -TIBIA_LENGTH,
    30
};
const static POINT maxDown = {
    FEMUR_COXA_SUM,
    -TIBIA_LENGTH,
    -maxZDown
};
const static POINT halfDown = {
    FEMUR_COXA_SUM,
    -TIBIA_LENGTH,
    -30
};
const static POINT legOutstretshed = {
    COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH,
    0,
    0
};
const static MOTION initializingMotion = {
    {initialPoint,initialPoint,initialPoint,initialPoint,initialPoint,initialPoint}, //all legs start at the initial position
    {neoInitial,neoInitial,neoInitial,neoInitial,neoInitial,neoInitial}, //all legs end at the neo position except leg 1
    {LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY}
};
const static MOTION legRaise = {
    {initialPoint,initialPoint,initialPoint,initialPoint,initialPoint,initialPoint}, //all legs start at the initial position
    {legOutstretshed,legOutstretshed,legOutstretshed,legOutstretshed,legOutstretshed,legOutstretshed}, //all legs end at the initial position except leg 1
    {LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY}
};
const static MOTION legReturn = {
    {legOutstretshed,legOutstretshed,legOutstretshed,legOutstretshed,legOutstretshed,legOutstretshed}, //all legs end at the initial position except leg 1
    {initialPoint,initialPoint,initialPoint,initialPoint,initialPoint,initialPoint}, //all legs start at the initial position
    {LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY,LINEAR_TRAJECTORY}
};
const static MOTION debugMotion1 = {
    {initialPoint,initialPoint,initialPoint,initialPoint,initialPoint,initialPoint}, //all legs start at the initial position
    {halfUP,halfDown,halfUP,halfDown,halfUP,halfDown},
    {LINEAR_ARC_TRAJECTORY,ARC_TRAJECTORY,LINEAR_ARC_TRAJECTORY,ARC_TRAJECTORY,LINEAR_ARC_TRAJECTORY,ARC_TRAJECTORY}
};
const static MOTION debugMotion2 = {
    {halfUP,halfDown,halfUP,halfDown,halfUP,halfDown},
    {initialPoint,initialPoint,initialPoint,initialPoint,initialPoint,initialPoint}, //all legs start at the initial position
    {ARC_TRAJECTORY,LINEAR_ARC_TRAJECTORY,ARC_TRAJECTORY,LINEAR_ARC_TRAJECTORY,ARC_TRAJECTORY,LINEAR_ARC_TRAJECTORY}
};
const static MOTION moveForward0 = {
     {initialPoint,initialPoint,initialPoint,initialPoint,initialPoint,initialPoint}, //all legs start at the initial position
     {forwardPoint,backwardPoint,forwardPoint,backwardPoint,forwardPoint,backwardPoint},
     {ARC_TRAJECTORY,LINEAR_TRAJECTORY,ARC_TRAJECTORY,LINEAR_TRAJECTORY,ARC_TRAJECTORY,LINEAR_TRAJECTORY}
};
const static MOTION moveForward1 = {
    {forwardPoint,backwardPoint,forwardPoint,backwardPoint,forwardPoint,backwardPoint},
    {backwardPoint,forwardPoint,backwardPoint,forwardPoint,backwardPoint,forwardPoint},
    {LINEAR_TRAJECTORY,ARC_TRAJECTORY,LINEAR_TRAJECTORY,ARC_TRAJECTORY,LINEAR_TRAJECTORY,ARC_TRAJECTORY}
};
const static MOTION moveForward2 = {
     {backwardPoint,forwardPoint,backwardPoint,forwardPoint,backwardPoint,forwardPoint},
     {initialPoint,initialPoint,initialPoint,initialPoint,initialPoint,initialPoint}, //all legs start at the initial position
     {ARC_TRAJECTORY,LINEAR_TRAJECTORY,ARC_TRAJECTORY,LINEAR_TRAJECTORY,ARC_TRAJECTORY,LINEAR_TRAJECTORY}
};
//prevents hexapod from moving
const static MOTION bufferMotion = {
    {0}, //all legs start at the initial position
    {0}, //all legs end at the initial position except leg 1
    {NONE_TRAJECTORY,NONE_TRAJECTORY,NONE_TRAJECTORY,NONE_TRAJECTORY,NONE_TRAJECTORY,NONE_TRAJECTORY}
};
//default configs for each link type, will be later removed
#define DEFAULT_COXA(pin) {90,90, COXA_MIN, COXA_MAX,pin}
#define DEFAULT_FEMUR(pin) {90,90, FEMUR_MIN, FEMUR_MAX,pin}
#define DEFAULT_TIBIA(pin) {90,90, TIBIA_MIN, TIBIA_MAX,pin}


#endif