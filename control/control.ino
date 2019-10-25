#include "PID_v1.h"
#include "PID_AutoTune_v0.h"

#define TIME_TO_STEP 10
#define MAX_ITER 100
#define STEP 2
#define PID_CONTROL 1
#define HALF_PULSE 5
#define PIN_OUTPUT 7

double Setpoint, Input, Output;
double Kp=5, Ki=5, Kd=5;
int counter;
bool up = true;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID_ATune aTune(&Input, &Output);

void setup() {
  Serial.begin(9600);
  Serial.println("setup() called.");
  Input = 0;
  counter = 0;
  Setpoint = 100;

  aTune.SetOutputStep(STEP); // How far above and below the starting value the output will step
  aTune.SetControlType(PID_CONTROL);
  aTune.SetLookbackSec(HALF_PULSE); // Look back for 5 seconds at previous data
  myPID.SetMode(AUTOMATIC); // Note: may want to look into manual mode, not sure how it works
}

void loop() {
  Serial.println("loop() called.");
  // TODO: Replace this junk with a call to the IMU for input
  counter++;
  if (counter >= MAX_ITER) {
    up = !up;
    counter == 0;
  } else if (counter % TIME_TO_STEP == 0) {
    Input += (up) ? 1 : -1;
  }
  
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);

  // Autotuning computation
  if (aTune.Runtime() != 0) {
    Serial.println("*** Got Params! ***");
    Kp = aTune.GetKp();
    Ki = aTune.GetKi();
    Kd = aTune.GetKd();
    char* message;
    sprintf(message, "Kp: %f   Ki: %f,  Kd: %f", Kp, Ki, Kd);
    Serial.println(message);
  }

  // TODO: Use the output to control the motor

  char* message;
  sprintf(message, "Input: %f     Output: %f", Input, Output);
  Serial.println(message);
}
