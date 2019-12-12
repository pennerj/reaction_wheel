#include "PID_v1.h"
#include "PID_AutoTune_v0.h"
#include "SparkFunMPU9250-DMP.h"

#define TIME_TO_STEP 10
#define MAX_ITER 100
#define STEP 2
#define PID_CONTROL 1
#define HALF_PULSE 5
#define PIN_OUTPUT 7
#define IMU_FREQ 100

double Setpoint, Input, Output;
double Kp=5, Ki=5, Kd=5;
int counter;
bool up = true;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID_ATune aTune(&Input, &Output);
MPU9250_DMP imu;
float oldGyroX = 0, oldGyroY = 0, oldGyroZ = 0;

void setup() {
  // Init Serial Monitor, variables, etc.
  Serial.begin(9600);
  Serial.println("setup() called.");
  Input = 0;
  counter = 0;
  Setpoint = 100;

  // Initialize the IMU
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_COMPASS);
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  imu.setLPF(42); // Set LPF corner frequency to 42Hz
  imu.setSampleRate(IMU_FREQ); // Set sample rate to 100Hz
  imu.setCompassSampleRate(IMU_FREQ); // Set mag rate to 100Hz
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               10); // Set DMP FIFO rate to 10 Hz

  // TODO: uncomment this to hopefully include all the code we need for PID control
  // aTune.SetOutputStep(STEP); // How far above and below the starting value the output will step
  // aTune.SetControlType(PID_CONTROL);
  // aTune.SetLookbackSec(HALF_PULSE); // Look back for 5 seconds at previous data
  // myPID.SetMode(AUTOMATIC); // Note: may want to look into manual mode, not sure how it works
}

void loop() {
  //Serial.println("loop() called.");
  if ( imu.dataReady() ) {
    imu.update(UPDATE_GYRO | UPDATE_COMPASS);
    // get the new IMU angle using w and the equation (dtheta / dt) * (1 / freq) = dtheta
    float gyroX = (imu.calcGyro(imu.gx) - oldGyroX / IMU_FREQ);
    float gyroY = (imu.calcGyro(imu.gy) - oldGyroY / IMU_FREQ);
    float gyroZ = (imu.calcGyro(imu.gz) - oldGyroZ / IMU_FREQ);
    float compass = imu.computeCompassHeading();
    
    Serial.print("Gyro x: ");
    Serial.print(gyroX, 6);
    Serial.print(" Gyro y: ");
    Serial.print(gyroY, 6);
    Serial.print(" Gyro z: ");
    Serial.print(gyroZ, 6);
    Serial.print(" Compass: ");
    Serial.println(compass, 6);

    // Keep track of old data for next loop
    oldGyroX = gyroX;
    oldGyroY = gyroY;
    oldGyroZ = gyroZ;
    counter++;
  }
  
  // TODO: More commented out PID control code, untested
  // myPID.Compute();
  // analogWrite(PIN_OUTPUT, Output);

  // Autotuning computation
  // if (aTune.Runtime() != 0) {
  //   Serial.println("*** Got Params! ***");
  //   Kp = aTune.GetKp();
  //   Ki = aTune.GetKi();
  //   Kd = aTune.GetKd();
  //   char* message;
  //   sprintf(message, "Kp: %f   Ki: %f,  Kd: %f", Kp, Ki, Kd);
  //   Serial.println(message);
  // }

  // TODO: Use the output to control the motor
}
