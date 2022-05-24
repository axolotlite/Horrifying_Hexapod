#ifndef _DEBUG_
#define _DEBUG_
//they are, as this header explains, for debugging purposes
//These flags determine wether serial prints are compiled or ignored during upload
static bool legInverseKinematicsFirstDebugFlag = false;
static bool legInverseKinematicsSecondDebugFlag = false;
static bool linearTrajectoryFlag = false;
static bool arcTrajectoryFlag = true;
static bool elipticalTrajectoryFlag = false;
static bool motionProcessFlag = false;
static bool updateAnglesFlag = true;

#endif