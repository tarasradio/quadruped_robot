#include <Arduino.h>
#include <IRremote.h>

#include <movements.h>
#include <quadroped.h>

IRrecv irrecv(11);
decode_results results;

// ----         ----
// | 6  | Face | 2  |
//  ---- -----  ----
//      | 7  3 |
// Left |      | Right
//      | 8  4 |
//  ---- ------ ----
// | 9  | Back | 5  |
// ----         ----

Quadroped quadroped;

int currentProgram = 0;

void handlePrograms();
void handleButtons(unsigned long buttonCode);

void setup() {
  irrecv.enableIRIn(); // Start the receiver
  Serial.begin(9600);
  Serial.println("Qudruped is ready!");

  quadroped.init();
  quadroped.goHome();
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    
    handleButtons(results.value);
    
    irrecv.resume(); // Receive the next value
  }
  handlePrograms();
  quadroped.tick();
}

void handleButtons(unsigned long buttonCode) {
  if(buttonCode == 0xFF38C7) { // 5
    currentProgram = 1;
  } else if(buttonCode == 0xFF18E7) { // 2
    currentProgram = 2;
  } else if(buttonCode == 0xFF4AB5) { // 8
    currentProgram = 3;
  } else if(buttonCode == 0xFF10EF) { // 4
    currentProgram = 6;
  } else if(buttonCode == 0xFF5AA5) { // 6
    currentProgram = 7;
  } else if(buttonCode == 0xFF30CF) { // 1
    currentProgram = 4;
  } else if(buttonCode == 0xFF7A85) { // 3
    currentProgram = 5;
  } else if(buttonCode == 0xFF6897) { // 0
    currentProgram = 9;
  } else if(buttonCode == 0xFFA25D) { // CH-
    currentProgram = 13;
  } else if(buttonCode == 0xFF629D) { // CH
    currentProgram = 14;
  } else if(buttonCode == 0xFFE21D) { // CH+
    currentProgram = 15;
  }
}

void handlePrograms() {
  if (currentProgram >= 1 ) {
    switch (currentProgram) {
      case 1: // Standby
        quadroped.runProgram(programStandby, programStandbySteps);
        break;
      case 2: // Forward
        quadroped.runProgram(programForward, programForwardSteps);
        break;
      case 3: // Backward
        quadroped.runProgram(programBackward, programBackwardSteps);
        break;
      case 4: // Left shift
        quadroped.runProgram(programLeftShift, programLeftShiftSteps);
        break;
      case 5: // Right shift
        quadroped.runProgram(programRightShift, programRightShiftSteps);
        break;
      case 6: // Turn left
        quadroped.runProgram(programTurnLeft, programTurnLeftSteps);
        break;
      case 7: // Turn right
        quadroped.runProgram(programTurnRight, programTurnRightSteps);
        break;
      case 8: // Lie
        quadroped.runProgram(programLie, programLieSteps);
        break;
      case 9: // Say Hi
        quadroped.runProgram(programSayHi, programSayHiSteps);
        quadroped.runProgram(programStandby, programStandbySteps);
        break;
      case 10: // Fighting
        quadroped.runProgram(programFighting, programFightingSteps);
        quadroped.runProgram(programStandby, programStandbySteps);
        break;
      case 11: // Push up
        quadroped.runProgram(programPushUp, programPushUpSteps);
        break;
      case 12: // Sleep
        quadroped.runProgram(programStandby, programStandbySteps);
        quadroped.runProgram(programSleep, programSleepSteps);
        break;
      case 13: // Dance 1
        quadroped.runProgram(programDance1, programDance1Steps);
        quadroped.runProgram(programStandby, programStandbySteps);
        break;
      case 14: // Dance 2
        quadroped.runProgram(programDance2, programDance2Steps);
        quadroped.runProgram(programStandby, programStandbySteps);
        break;
      case 15: // Dance 3
        quadroped.runProgram(programDance3, programDance3Steps);
        quadroped.runProgram(programStandby, programStandbySteps);
        break;
    }
    //currentProgram = 0;
  }
}