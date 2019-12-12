# reaction_wheel
Software for the BYU Spacecraft Attitude Control System team

Authors: Jordan Penner and Wesley Stirk

Email: pennerj66@gmail.com

Breakdown of the file system:

ESC Controller: Originally we got our motors to spin using a hobby ESC controller. Since then we've decided to switch to a controller that allows for better precision and is bi-directional, but the old code is provided for anyone interested. It worked the last time it was used, in May 2019.

SparkFun_MPU-9250-DMP_Arduino_Library: Library made for the SparkFun MPU-9250 Inertial Measurement Unit. Useful for communicating with an IMU to get acceleration and gyroscopic motion data.

br3ttb-Arduino-PID-AutoTune-Library-7c03cf3: Arduino Libraries for a PID controller and PID AutoTune controller. The PID controller provides a general PID algorithm. The PID AutoTune controller hooks up with the PID to help fine-tune the Kp, Kd, and Ki coefficients.

control: Our main controller. This is the interface between all the subsystems in the project. The IMU's data is the input to the PID controller, which then spits out some value which we would give to the motor driver to know how fast to go. UNTESTED

motor_sw: Software to control the motor driver specific to this project. UNTESTED
