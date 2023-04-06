#include <Arduino.h>
#include <Servo.h>
#include <IRremote.h>

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

Servo servos[8];

uint8_t servos_pins[8] = { 2, 3, 4, 5, 6, 7, 8, 9 };
int8_t servos_offsets[8] = { 5, -9, -8, -8, 0, -12, -2, -8 };
uint8_t servos_positions[8] = { 90, 90, 90, 90, 90, 90, 90, 90 };

// Servos matrix
const int ALLMATRIX = 9; // + Run Time
const int ALLSERVOS = 8; //

// Action
// --------------------------------------------------------------------------------

// Servo zero position 
// ----------------------------- G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
const int16_t Servo_Act_0 [ ] PROGMEM = {  135,  45, 135,  45,  45, 135,  45, 135,  500  };

// Start position 
// ----------------------------- G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
const int16_t Servo_Act_1 [ ] PROGMEM = {  135,  45, 135,  45,  45, 135,  45, 135,  500  };

// Standby 
int programStandbySteps = 2;
const int16_t programStandby [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  500  }, // servo center point
  {   70,  90,  90, 110, 110,  90,  90,  70,  500  }, // standby
};

// Forward 
int programForwardSteps = 11;
const int16_t programForward [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  150  }, // standby
  {   90,  90,  90, 110, 110,  90,  45,  90,  150  }, // leg1,4 up; leg4 fw
  {   70,  90,  90, 110, 110,  90,  45,  70,  150  }, // leg1,4 dn
  {   70,  90,  90,  90,  90,  90,  45,  70,  150  }, // leg2,3 up
  {   70,  45, 135,  90,  90,  90,  90,  70,  150  }, // leg1,4 bk; leg2 fw
  {   70,  45, 135, 110, 110,  90,  90,  70,  150  }, // leg2,3 dn
  {   90,  90, 135, 110, 110,  90,  90,  90,  150  }, // leg1,4 up; leg1 fw
  {   90,  90,  90, 110, 110, 135,  90,  90,  150  }, // leg2,3 bk
  {   70,  90,  90, 110, 110, 135,  90,  70,  150  }, // leg1,4 dn
  {   70,  90,  90, 110,  90, 135,  90,  70,  150  }, // leg3 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  150  }, // leg3 fw dn
};

// Backward 
int programBackwardSteps = 11;
const int16_t programBackward [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  150  }, // standby
  {   90,  45,  90, 110, 110,  90,  90,  90,  150  }, // leg4,1 up; leg1 fw
  {   70,  45,  90, 110, 110,  90,  90,  70,  150  }, // leg4,1 dn
  {   70,  45,  90,  90,  90,  90,  90,  70,  150  }, // leg3,2 up
  {   70,  90,  90,  90,  90, 135,  45,  70,  150  }, // leg4,1 bk; leg3 fw
  {   70,  90,  90, 110, 110, 135,  45,  70,  150  }, // leg3,2 dn
  {   90,  90,  90, 110, 110, 135,  90,  90,  150  }, // leg4,1 up; leg4 fw
  {   90,  90, 135, 110, 110,  90,  90,  90,  150  }, // leg3,1 bk
  {   70,  90, 135, 110, 110,  90,  90,  70,  150  }, // leg4,1 dn
  {   70,  90, 135,  90, 110,  90,  90,  70,  150  }, // leg2 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  150  }, // leg2 fw dn
};

// Left shift 
int programLeftShiftSteps = 11;
const int16_t programLeftShift [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  150  }, // standby
  {   70,  90,  50,  90,  90,  90,  90,  70,  150  }, // leg3,2 up; leg2 fw
  {   70,  90,  50, 110, 110,  90,  90,  70,  150  }, // leg3,2 dn
  {   90,  90,  50, 110, 110,  90,  90,  90,  150  }, // leg1,4 up
  {   90, 135,  90, 110, 110,  50,  90,  90,  150  }, // leg3,2 bk; leg1 fw
  {   70, 135,  90, 110, 110,  50,  90,  70,  150  }, // leg1,4 dn
  {   70, 135,  90,  90,  90,  90,  90,  70,  150  }, // leg3,2 up; leg3 fw
  {   70,  90,  90,  90,  90,  90, 135,  70,  150  }, // leg1,4 bk
  {   70,  90,  90, 110, 110,  90, 135,  70,  150  }, // leg3,2 dn
  {   70,  90,  90, 110, 110,  90, 135,  90,  150  }, // leg4 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  150  }, // leg4 fw dn
};

// Right shift 
int programRightShiftSteps = 11;
const int16_t programRightShift [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  150  }, // standby
  {   70,  90,  90,  90,  90,  50,  90,  70,  150  }, // leg2,3 up; leg3 fw
  {   70,  90,  90, 110, 110,  50,  90,  70,  150  }, // leg2,3 dn
  {   90,  90,  90, 110, 110,  50,  90,  90,  150  }, // leg4,1 up
  {   90,  90,  50, 110, 110,  90, 135,  90,  150  }, // leg2,3 bk; leg4 fw
  {   70,  90,  50, 110, 110,  90, 135,  70,  150  }, // leg4,1 dn
  {   70,  90,  90,  90,  90,  90, 135,  70,  150  }, // leg2,3 up; leg2 fw
  {   70, 135,  90,  90,  90,  90,  90,  70,  150  }, // leg4,1 bk
  {   70, 135,  90, 110, 110,  90,  90,  70,  150  }, // leg2,3 dn
  {   90, 135,  90, 110, 110,  90,  90,  70,  150  }, // leg1 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  150  }, // leg1 fw dn
};

// Turn left 
int programTurnLeftSteps = 8;
const int16_t programTurnLeft [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  150  }, // standby
  {   90,  90,  90, 110, 110,  90,  90,  90,  150  }, // leg1,4 up
  {   90, 135,  90, 110, 110,  90, 135,  90,  150  }, // leg1,4 turn
  {   70, 135,  90, 110, 110,  90, 135,  70,  150  }, // leg1,4 dn
  {   70, 135,  90,  90,  90,  90, 135,  70,  150  }, // leg2,3 up
  {   70, 135, 135,  90,  90, 135, 135,  70,  150  }, // leg2,3 turn
  {   70, 135, 135, 110, 110, 135, 135,  70,  150  }, // leg2,3 dn
  {   70,  90,  90, 110, 110,  90,  90,  70,  150  }, // leg1,2,3,4 turn
};

// Turn right 
int programTurnRightSteps = 8;
const int16_t programTurnRight [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  150  }, // standby
  {   70,  90,  90,  70,  70,  90,  90,  70,  150  }, // leg2,3 up
  {   70,  90,  50,  70,  70,  50,  90,  70,  150  }, // leg2,3 turn
  {   70,  90,  50, 110, 110,  50,  90,  70,  150  }, // leg2,3 dn
  {   90,  90,  50, 110, 110,  50,  90,  90,  150  }, // leg1,4 up
  {   90,  50,  50, 110, 110,  50,  50,  90,  150  }, // leg1,4 turn
  {   70,  50,  50, 110, 110,  50,  50,  70,  150  }, // leg1,4 dn
  {   70,  90,  90, 110, 110,  90,  90,  70,  150  }, // leg1,2,3,4 turn
};

// Lie 
int programLieSteps = 1;
const int16_t programLie [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  110,  90,  90,  70,  70,  90,  90, 110,  500  }, // leg1,2,3,4 up
};

// Say Hi 
int programSayHiSteps = 11;
const int16_t programSayHi [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  120,  90,  90, 110,  60,  90,  90,  70,  400}, // leg1, 3 down
  {   70,  90,  90, 110, 110,  90,  90,  70,  400}, // standby
  {  120,  90,  90, 110,  60,  90,  90,  70,  400}, // leg1, 3 down
  {   70,  90,  90, 110, 110,  90,  90,  70,  400}, // standby
    
  {   70,  90,  90,  90,  90,  50,  90,  90,  400}, // leg2,3,4 dn
  {  170,  90,  90,  90,  90,  50,  90,  90,  400}, // leg1 up
  {  170, 130,  90,  90,  90,  50,  90,  90,  400}, // leg1 left
  {  170,  50,  90,  90,  90,  50,  90,  90,  400}, // leg1 right
  {  170, 130,  90,  90,  90,  50,  90,  90,  400}, // leg1 left
  {  170,  90,  90,  90,  90,  50,  90,  90,  400}, // leg1 right
  {   70,  90,  90,  90,  90,  50,  90,  90,  400}, // leg1 dn
};

// Fighting (fighting stance)
int programFightingSteps = 11;
const int16_t programFighting [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {  120,  90,  90, 110,  60,  90,  90,  70,  500  }, // leg1, 2 down
  {  120,  70,  70, 110,  60,  70,  70,  70,  500  }, // body turn left
  {  120, 110, 110, 110,  60, 110, 110,  70,  500  }, // body turn right
  {  120,  70,  70, 110,  60,  70,  70,  70,  500  }, // body turn left
  {  120, 110, 110, 110,  60, 110, 110,  70,  500  }, // body turn right
  {   70,  90,  90,  70, 110,  90,  90, 110,  500  }, // leg1, 2 up ; leg3, 4 down
  {   70,  70,  70,  70, 110,  70,  70, 110,  500  }, // body turn left
  {   70, 110, 110,  70, 110, 110, 110, 110,  500  }, // body turn right
  {   70,  70,  70,  70, 110,  70,  70, 110,  500  }, // body turn left
  {   70, 110, 110,  70, 110, 110, 110, 110,  500  }, // body turn right
  {   70,  90,  90,  70, 110,  90,  90, 110,  500  }  // leg1, 2 up ; leg3, 4 down
};

// Push up
int programPushUpSteps = 11;
const int16_t programPushUp [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  500  }, // start
  {  100,  90,  90,  80,  80,  90,  90, 100,  600  }, // down
  {   70,  90,  90, 110, 110,  90,  90,  70,  700  }, // up
  {  100,  90,  90,  80,  80,  90,  90, 100,  800  }, // down
  {   70,  90,  90, 110, 110,  90,  90,  70,  900  }, // up
  {  100,  90,  90,  80,  80,  90,  90, 100, 1500  }, // down
  {   70,  90,  90, 110, 110,  90,  90,  70, 2000  }, // up
  {  135,  90,  90,  45,  45,  90,  90, 135,  200  }, // fast down
  {   70,  90,  90,  45,  60,  90,  90, 135,  800  }, // leg1 up
  {   70,  90,  90,  45, 110,  90,  90, 135,  800  }, // leg2 up
  {   70,  90,  90, 110, 110,  90,  90,  70,  800  }  // leg3, leg4 up
};

// Sleep (sleeping position)
int programSleepSteps = 2;
const int16_t programSleep [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  500  }, // leg1,4 dn
  {   30,  45, 135, 150, 150, 135,  45,  30,  500  }, // protect myself
};

// Dance steps 1
int ProgramDance1Steps = 10;
const int16_t ProgramDance1 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  400  }, // leg1,2,3,4 up
  {   50,  90,  90,  90,  90,  90,  90,  90,  400  }, // leg1 dn
  {   90,  90,  90, 130,  90,  90,  90,  90,  400  }, // leg1 up; leg2 dn
  {   90,  90,  90,  90,  90,  90,  90,  50,  400  }, // leg2 up; leg4 dn
  {   90,  90,  90,  90, 130,  90,  90,  90,  400  }, // leg4 up; leg3 dn
  {   50,  90,  90,  90,  90,  90,  90,  90,  400  }, // leg3 up; leg1 dn
  {   90,  90,  90, 130,  90,  90,  90,  90,  400  }, // leg1 up; leg2 dn
  {   90,  90,  90,  90,  90,  90,  90,  50,  400  }, // leg2 up; leg4 dn
  {   90,  90,  90,  90, 130,  90,  90,  90,  400  }, // leg4 up; leg3 dn
  {   90,  90,  90,  90,  90,  90,  90,  90,  400  }, // leg3 up
};

// Dance steps 2
int ProgramDance2Steps = 9;
const int16_t ProgramDance2 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   70,  45, 135, 110, 110, 135,  45,  70,  400  }, // leg1,2,3,4 two sides
  {  115,  45, 135,  65, 110, 135,  45,  70,  400  }, // leg1,2 up
  {   70,  45, 135, 110,  65, 135,  45, 115,  400  }, // leg1,2 dn; leg3,4 up
  {  115,  45, 135,  65, 110, 135,  45,  70,  400  }, // leg3,4 dn; leg1,2 up
  {   70,  45, 135, 110,  65, 135,  45, 115,  400  }, // leg1,2 dn; leg3,4 up
  {  115,  45, 135,  65, 110, 135,  45,  70,  400  }, // leg3,4 dn; leg1,2 up
  {   70,  45, 135, 110,  65, 135,  45, 115,  400  }, // leg1,2 dn; leg3,4 up
  {  115,  45, 135,  65, 110, 135,  45,  70,  400  }, // leg3,4 dn; leg1,2 up
  {   75,  45, 135, 105, 110, 135,  45,  70,  400  }, // leg1,2 dn
};

// Dance steps 3
int ProgramDance3Steps = 10;
const int16_t ProgramDance3 [][ALLMATRIX] PROGMEM = {
  // G14, G12, G13, G15, G16,  G5,  G4,  G2,  ms
  {   70,  50,  50, 110, 110, 135, 135,  70,  400  }, // leg1,2,3,4 bk
  {  110,  50,  50,  60,  70, 135, 135,  70,  400  }, // leg1,2,3 up
  {   70,  50,  50, 110, 110, 135, 135,  70,  400  }, // leg1,2,3 dn
  {  110,  50,  50, 110,  70, 135, 135, 120,  400  }, // leg1,3,4 up
  {   70,  50,  50, 110, 110, 135, 135,  70,  400  }, // leg1,3,4 dn
  {  110,  50,  50,  60,  70, 135, 135,  70,  400  }, // leg1,2,3 up
  {   70,  50,  50, 110, 110, 135, 135,  70,  400  }, // leg1,2,3 dn
  {  110,  50,  50, 110,  70, 135, 135, 120,  400  }, // leg1,3,4 up
  {   70,  50,  50, 110, 110, 135, 135,  70,  400  }, // leg1,3,4 dn
  {   70,  90,  90, 110, 110,  90,  90,  70,  400  }, // standby
};

// --------------------------------------------------------------------------------

void servo_write(uint8_t s, int angle) {
  int const_angle = angle + servos_offsets[s];
  const_angle = constrain(const_angle, 0, 180);
  servos[s].write(const_angle);
}

void go_home() {
  for(int s = 0; s < 8; s++) {
    int angle = pgm_read_word(&Servo_Act_0[s]);
    servo_write(s, angle);
    servos_positions[s] = angle;
  }
}

#define TICK_PERIOD 10

void runProgram(const int16_t program[][ALLMATRIX], int programSteps) {
  int c_angle, t_angle, n_angle = 0;
  for(int i = 0 ; i < programSteps; i++) {
    int totalPeriod = pgm_read_word(&program[i][8]) ; // общее время выполнения итерации
    int totalPeriodCounter = totalPeriod / TICK_PERIOD; // число отсчетов (делим общее время на шаг по времени)

    for(int t = 0; t < totalPeriodCounter; t++) {
      for(int j = 0; j < 8; j++) {
        c_angle = servos_positions[j];
        t_angle = pgm_read_word(&program[i][j]);

        if(c_angle == t_angle) {
          n_angle = t_angle;
        } else if(c_angle > t_angle) {
          n_angle = map(t * TICK_PERIOD, 0, totalPeriod, 0, c_angle - t_angle);
          if(c_angle - n_angle >= t_angle) {
            servo_write(j, c_angle - n_angle);
          }
        } else if(c_angle < t_angle) {
          n_angle = map(t * TICK_PERIOD, 0, totalPeriod, 0, t_angle - c_angle);
          if(c_angle + n_angle <= t_angle) {
            servo_write(j, c_angle + n_angle);
          }
        }
      }
      delay(TICK_PERIOD);
    }

    for (int j = 0; j < 8; j++) {
      servos_positions[j] = pgm_read_word(&program[i][j]);
    }
  }
}

int currentProgram = 0;
void handlePrograms();
void handleButtons(unsigned long buttonCode);

void setup() {
  irrecv.enableIRIn(); // Start the receiver
  Serial.begin(9600);
  Serial.println("Qudruped is ready!");

  for(uint8_t i = 0; i < 8; i++) {
    servos[i].attach(servos_pins[i]);
  }

  go_home();
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    
    handleButtons(results.value);
    
    irrecv.resume(); // Receive the next value
  }

  handlePrograms();
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
        runProgram(programStandby, programStandbySteps);
        break;
      case 2: // Forward
        runProgram(programForward, programForwardSteps);
        break;
      case 3: // Backward
        runProgram(programBackward, programBackwardSteps);
        break;
      case 4: // Left shift
        runProgram(programLeftShift, programLeftShiftSteps);
        break;
      case 5: // Right shift
        runProgram(programRightShift, programRightShiftSteps);
        break;
      case 6: // Turn left
        runProgram(programTurnLeft, programTurnLeftSteps);
        break;
      case 7: // Turn right
        runProgram(programTurnRight, programTurnRightSteps);
        break;
      case 8: // Lie
        runProgram(programLie, programLieSteps);
        break;
      case 9: // Say Hi
        runProgram(programSayHi, programSayHiSteps);
        runProgram(programStandby, programStandbySteps);
        break;
      case 10: // Fighting
        runProgram(programFighting, programFightingSteps);
        runProgram(programStandby, programStandbySteps);
        break;
      case 11: // Push up
        runProgram(programPushUp, programPushUpSteps);
        break;
      case 12: // Sleep
        runProgram(programStandby, programStandbySteps);
        runProgram(programSleep, programSleepSteps);
        break;
      case 13: // Dance 1
        runProgram(ProgramDance1, ProgramDance1Steps);
        runProgram(programStandby, programStandbySteps);
        break;
      case 14: // Dance 2
        runProgram(ProgramDance2, ProgramDance2Steps);
        runProgram(programStandby, programStandbySteps);
        break;
      case 15: // Dance 3
        runProgram(ProgramDance3, ProgramDance3Steps);
        runProgram(programStandby, programStandbySteps);
        break;
    }
    currentProgram = 0;
  }
}