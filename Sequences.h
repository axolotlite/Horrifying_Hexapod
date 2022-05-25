
#ifndef _SEQUENCES_
#define _SEQUENCES_
#include "motion_core.h"

#define SUPPORTED_MOTIONS_COUNT 5

typedef enum{
    NONE_SEQUENCE,
    ROTATE_LEFT_SEQUENCE,
    ROTATE_RIGHT_SEQUENCE,
    FORWARD_SEQUENCE

}SEQUENCE_ID;

typedef struct {
    bool loopable;
    unsigned char motionsCount;
    MOTION motions[SUPPORTED_MOTIONS_COUNT];
}SEQUENCE;
const static SEQUENCE idleSeq = {
    false,
    1,
    {legRaise, legReturn}
};
const static SEQUENCE rotateRight = {
   false,
   2,
   {
       debugMotion1,
       debugMotion2
   }
};

#endif
