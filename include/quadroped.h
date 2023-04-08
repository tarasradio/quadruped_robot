#ifndef _QUADROPED_H_
#define _QUADROPED_H_

#include <Arduino.h>
#include <Servo.h>
#include <config.h>

#define TICK_PERIOD 10

class Quadroped {
public:
  Quadroped();
  void init();
  void goHome();
  void runProgram(const int16_t program[][PARAMS_SIZE], int programSteps);
  void tick();
private:
  Servo servos[8];

  uint8_t servos_pins[8] = { 2, 3, 4, 5, 6, 7, 8, 9 };
  int8_t servos_offsets[8] = { 5, -9, -8, -8, 0, -12, -2, -8 };
  uint8_t servos_positions[8] = { 90, 90, 90, 90, 90, 90, 90, 90 };

  int16_t const (*program)[PARAMS_SIZE];
  int programSteps;

  int programState = 0;
  int programStep = 0;

  long programStepTimer = 0;

  void servoWrite(uint8_t servo, int angle);
  void performProgramStep(int step);
  void performProgramStepTick(int step, int tick, int totalPeriod);
};

#endif // _QUADROPED_H_