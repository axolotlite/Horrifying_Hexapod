#include "Sequences.h"
#include "debug.h"
#include <Arduino.h>

typedef enum {
    IDLE_STATE,
    MOVE_STATE,
    WAIT_STATE,
    CHANGE_MOTION_STATE,
    CHANGE_SEQUENCE_STATE
}ENGINE_STATE;

static ENGINE_STATE currentState = IDLE_STATE;
static SEQUENCE_ID currentID;
static SEQUENCE currentSequence = idleSeq;
static SEQUENCE_ID nextID;
static SEQUENCE nextSequence = idleSeq;

//debug transition states, won't be used if the flag is set to false
static ENGINE_STATE transitions[SUPPORTED_TRANSITION_COUNT];
static unsigned char transitionCount = 0;

static unsigned char motionCount = 0;
static void sequenceEngineInit(){
    currentSequence = idleSeq;
    nextSequence = idleSeq;

}
//resets motions in case they'll be reused later
static void resetSequence(){
    for(int i=0;i<currentSequence.motionsCount ;i++)
        currentSequence.motions[i].isDone = false;
}
//needs a shit ton of debugging and debugging flags.
static void sequenceEngine(){
    //needs some debugging flags
    if(currentSequence.motionsCount >= SUPPORTED_MOTIONS_COUNT){
        return;
    }
    //need to add buffer between sequence change
    
    //engine states
    switch(currentState){
        case IDLE_STATE:
        //return when values for sequences have been defined
            if(currentID != nextID)
                currentState = CHANGE_MOTION_STATE;
            break;
        case MOVE_STATE:
            startNextMotion(&(currentSequence.motions[motionCount]));
            currentState = WAIT_STATE;
            break;
        //to be implemented later
        case WAIT_STATE:
            if(currentSequence.motions[motionCount].isDone){
                currentState = CHANGE_MOTION_STATE;
                //wait for one second 
                if(motionTransitionDelayFlag){
                    delay(1000);
                }
            }
            break;
        case CHANGE_MOTION_STATE:
            //may cause a bug where an entire motion is skipped
            motionCount++;
            if(motionCount >= currentSequence.motionsCount)
                currentState = CHANGE_SEQUENCE_STATE;
            else 
                currentState = MOVE_STATE;
            break;
        case CHANGE_SEQUENCE_STATE:
            //this may cause a disaster of unimaginable magnitude, because it's time dependent and we can't imagine how time could cause the disaster
            //check seq num has exceeded current sequences, if so; null.
            //the current and next sequences are made equal
            currentSequence = nextSequence;
            //move to the idle state, where it will remain there deadlocked, until sequenceSelector, changes the next sequence.
            currentState = IDLE_STATE;
        default:break;
    }
    if(currentStateFlag){
        char* currentStateString;
        switch(currentState){
            case IDLE_STATE: currentStateString = "IDLE_STATE";break;
            case MOVE_STATE: currentStateString = "MOVE_STATE";break;
            case WAIT_STATE: currentStateString = "WAIT_STATE";break;
            case CHANGE_MOTION_STATE: currentStateString = "CHANGE_MOTION_STATE";break;
            case CHANGE_SEQUENCE_STATE: currentStateString = "CHANGE_SEQUENCE_STATE";break;
            default: currentStateString = "DEFAULT_STATE";break;
        }
        Serial.println("Current state: ");Serial.println(currentStateString);
    }
    if(stateTransitionFlag){
        if(transitions[transitionCount] != currentState)
            transitions[transitionCount] = currentState;
        //once a repeated state is reached, it'll print the output 
        else{
            char* currentStateString;
            for(unsigned char i=0;i<transitionCount;i++){
                switch(currentState){
                    case IDLE_STATE: currentStateString = "IDLE_STATE";break;
                    case MOVE_STATE: currentStateString = "MOVE_STATE";break;
                    case WAIT_STATE: currentStateString = "WAIT_STATE";break;
                    case CHANGE_MOTION_STATE: currentStateString = "CHANGE_MOTION_STATE";break;
                    case CHANGE_SEQUENCE_STATE: currentStateString = "CHANGE_SEQUENCE_STATE";break;
                    default: currentStateString = "DEFAULT_STATE";break;
                }
                Serial.print("Current state->");Serial.print(currentStateString);
            }
            Serial.println();
            return;
        }
        if(transitionCount >= SUPPORTED_TRANSITION_COUNT){
            Serial.println("Supported transitions array full");
        }
        else transitionCount++;
    }

}

static void sequenceSelector(SEQUENCE_ID ID){
    //we need to point to the sequence instead of taking a copy of it.
    nextID = ID;
    switch(ID){
        case NONE_SEQUENCE : break;
        case ROTATE_LEFT_SEQUENCE : break;
        case ROTATE_RIGHT_SEQUENCE : nextSequence = rotateRight; break;
        case FORWARD_SEQUENCE : break;
        default:break;
    }
    if(currentSequenceFlag){
        char* currentSequenceString;
        switch(ID){
            case NONE_SEQUENCE : currentSequenceString = "NONE_SEQUENCE"; break;
            case ROTATE_LEFT_SEQUENCE : currentSequenceString = "ROTATE_LEFT_SEQUENCE"; break;
            case ROTATE_RIGHT_SEQUENCE : currentSequenceString = "ROTATE_RIGHT_SEQUENCE"; break;
            case FORWARD_SEQUENCE : currentSequenceString = "FORWARD_SEQUENCE"; break;
            default: currentSequenceString = "DEFAULT_STATE";break;
        }
        Serial.println("Current Sequence: ");Serial.println(currentSequenceString);
    }
}