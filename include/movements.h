#ifndef MOVEMENTS_H
#define MOVEMENTS_H

#include <Arduino.h>
#include <config.h>

// Servo zero position
extern const int16_t Servo_Act_0 [];

// Start position
extern const int16_t Servo_Act_1 [];

// Standby 
extern int programStandbySteps;
extern const int16_t programStandby [][PARAMS_SIZE];

// Forward 
extern int programForwardSteps;
extern const int16_t programForward [][PARAMS_SIZE];

// Backward 
extern int programBackwardSteps;
extern const int16_t programBackward [][PARAMS_SIZE];

// Left shift 
extern int programLeftShiftSteps;
extern const int16_t programLeftShift [][PARAMS_SIZE];

// Right shift 
extern int programRightShiftSteps;
extern const int16_t programRightShift [][PARAMS_SIZE];

// Turn left 
extern int programTurnLeftSteps;
extern const int16_t programTurnLeft [][PARAMS_SIZE];

// Turn right 
extern int programTurnRightSteps;
extern const int16_t programTurnRight [][PARAMS_SIZE];

// Lie 
extern int programLieSteps;
extern const int16_t programLie [][PARAMS_SIZE];

// Say Hi 
extern int programSayHiSteps;
extern const int16_t programSayHi [][PARAMS_SIZE];

// Fighting (fighting stance)
extern int programFightingSteps;
extern const int16_t programFighting [][PARAMS_SIZE];

// Push up
extern int programPushUpSteps;
extern const int16_t programPushUp [][PARAMS_SIZE];

// Sleep (sleeping position)
extern int programSleepSteps;
extern const int16_t programSleep [][PARAMS_SIZE];

// Dance steps 1
extern int programDance1Steps;
extern const int16_t programDance1 [][PARAMS_SIZE];

// Dance steps 2
extern int programDance2Steps;
extern const int16_t programDance2 [][PARAMS_SIZE];

// Dance steps 3
extern int programDance3Steps;
extern const int16_t programDance3 [][PARAMS_SIZE];

#endif // MOVEMENTS_H