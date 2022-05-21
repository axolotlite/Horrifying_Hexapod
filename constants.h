#ifndef _CONSTANTS_
#define _CONSTANTS_

#define LIMB_COUNT 6
#define STEP_HEIGHT 30
//The following measurements are in mm
#define SIDE_LENGTH 55.57

// #define COXA_LENGTH 26.24
// #define FEMUR_LENGTH 40.1
// #define TIBIA_LENGTH 80.13

#define COXA_LENGTH 26
#define COXA_LENGTH_SQUARED 676
#define FEMUR_LENGTH 40
#define FEMUR_LENGTH_SQUARED 1600
#define TIBIA_LENGTH 80
#define TIBIA_LENGTH_SQUARED 6400
#define FEMUR_COXA_SUM 66

#define GYRO_COXA 10.08

#define GYRO_GROUND 75.46

// min and max angles of each leg
#define COXA_MIN 25
#define COXA_MAX 165

#define DEFAULT_COXA(pin) {90,90, COXA_MIN, COXA_MAX,pin}
#define DEFAULT_FEMUR(pin) {90,90, 0, 180,pin}
#define DEFAULT_TIBIA(pin) {90,90, 0, 180,pin}

#define STEP_TIME 10
#define SCALE_TIME 1000 //will need to be cast into unsigned int


#endif
