#ifndef _SERVO_DRIVER_CONFIGS
#define  _SERVO_DRIVER_CONFIGS

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  500 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2300 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define MAXROTATION 180 // the maximum rotation applicable for this servo
//#define PWM_2_DEGREE(angle) (USMIN + angle*round((USMAX*1.0f - USMIN)/MAXROTATION*1.0f))  // each step takes one of these bad boys
#define PWM_2_DEGREE(angle) (USMIN + angle*10) 

#endif