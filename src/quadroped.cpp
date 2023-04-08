#include <quadroped.h>
#include <movements.h>

Quadroped::Quadroped() {

}

void Quadroped::init() {
  for(uint8_t i = 0; i < 8; i++) {
    servos[i].attach(servos_pins[i]);
  }
}

void Quadroped::servoWrite(uint8_t servo, int angle) {
  int const_angle = angle + servos_offsets[servo];
  const_angle = constrain(const_angle, 0, 180);
  servos[servo].write(const_angle);
}

void Quadroped::goHome() {
  for(int i = 0; i < 8; i++) {
    int angle = pgm_read_word(&Servo_Act_0[i]);
    servoWrite(i, angle);
    servos_positions[i] = angle;
  }
}

void Quadroped::runProgram(const int16_t program[][PARAMS_SIZE], int programSteps) {
  if(programState == 0) {
    programState = 1;

    this->program = program;
    this->programSteps = programSteps;

    programStep = 0;
    programStepTimer = millis();
  }

  // for(int i = 0 ; i < programSteps; i++) {
  //   performProgramStep(i);
  // }
}

void Quadroped::tick() {
  static uint64_t timer = millis();
  if(millis() - timer >= TICK_PERIOD) {
    timer = millis();

    if(programState != 0) { // program is running
      int totalPeriod = pgm_read_word(&program[programStep][8]);
      int t = millis() - programStepTimer;

      if(t <= totalPeriod) { // current step does not finished
        performProgramStepTick(programStep, t, totalPeriod);
      } else { // current step finidhed, switch next step
        for (int i = 0; i < 8; i++) {
          servoWrite(i, pgm_read_word(&program[programStep][i]));
          servos_positions[i] = pgm_read_word(&program[programStep][i]);
        }
        programStep++;
        programStepTimer = millis();
        if (programStep == programSteps) {
          programState = 0; // program finished
        }
      }
    }
  }
}

void Quadroped::performProgramStep(int step) {
  int totalPeriod = pgm_read_word(&program[step][8]) ; // общее время выполнения итерации
  int totalPeriodCounter = totalPeriod / TICK_PERIOD; // число отсчетов (делим общее время на шаг по времени)

  for(int t = 0; t < totalPeriodCounter; t++) {
    performProgramStepTick(step, t, totalPeriod);
  }

  for (int j = 0; j < 8; j++) {
    servos_positions[j] = pgm_read_word(&program[step][j]);
  }
}

void Quadroped::performProgramStepTick(int step, int tick, int totalPeriod) {
  int c_angle, t_angle, n_angle = 0;

  long c_time = tick * TICK_PERIOD;

  for(int j = 0; j < 8; j++) {
    c_angle = servos_positions[j];
    t_angle = pgm_read_word(&program[step][j]);

    if(c_angle == t_angle) {
      n_angle = t_angle;
    } else if(c_angle > t_angle) {
      n_angle = map(c_time, 0, totalPeriod, 0, c_angle - t_angle);
      if(c_angle - n_angle >= t_angle) {
        servoWrite(j, c_angle - n_angle);
      }
    } else if(c_angle < t_angle) {
      n_angle = map(c_time, 0, totalPeriod, 0, t_angle - c_angle);
      if(c_angle + n_angle <= t_angle) {
        servoWrite(j, c_angle + n_angle);
      }
    }
  }
  //delay(TICK_PERIOD);
}