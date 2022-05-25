#ifndef _DEBUG_
#define _DEBUG_
//they are, as this header explains, for debugging purposes
//These flags determine wether serial prints are compiled or ignored during upload

//motion core debug flags
static bool legInverseKinematicsFirstDebugFlag = true;
static bool legInverseKinematicsSecondDebugFlag = false;
static bool linearTrajectoryFlag = true;
static bool arcTrajectoryFlag = false;
static bool elipticalTrajectoryFlag = false;
static bool motionProcessFlag = false;
static bool updateAnglesFlag = false;

//sequence engine debug flags
static bool motionTransitionDelayFlag = false;
static bool currentStateFlag = false;
static bool currentSequenceFlag = false;

static bool stateTransitionFlag = false;
#define SUPPORTED_TRANSITION_COUNT 21

#endif