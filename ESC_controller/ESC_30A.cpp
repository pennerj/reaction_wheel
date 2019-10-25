/*
 * Created on February 2, 2019 by Wesley Stirk
 * 
 * This file has the implementation of the basic members and functions needed to control the 30A BLDC ESC from Hobbypower
 * It relies heavily on the Servo library for much of its functionality. 
 * s
 * Last Updated February 27, 2019 by Wesley Stirk
 */

#include "ESC_30A.h"

ESC_30A::ESC_30A(uint8_t pin)
{
  this->pin = pin;
}

void ESC_30A::init(int8_t newPin)
{
  if(newPin >= 0) //if initialized with a different pin use that one. 
  {
    this->pin = newPin;
  }
  bldc.attach(pin);//attach the correct pin to the 'servo' 
    //causes issues if not called in the setup (or loop) function so it can't be called in the constructor
  full_stop(); //start of with the motor not moving
}

void ESC_30A::throttle(speed_t mSpeed)
{
  if(mSpeed <= ESC_30A_MAX_SPEED || mSpeed >= ESC_30A_MIN_SPEED) //double check that the given speed in the appropriate ranged
  {
    uint16_t rate = map(mSpeed, ESC_30A_MIN_SPEED, ESC_30A_MAX_SPEED, 0, 1000) + 1000;
    //map(mSpeed, ESC_30A_MIN_SPEED, ESC_30A_MAX_SPEED, 0, 180); //servos go from 0 to 180 so it needs to be mapped to that range.
    #ifdef ESC_DEBUG
      Serial.print("throttling to: ");
      Serial.println(rate);
    #endif
    //bldc.write(rate); //set the appropriate speed
    throttle_pulse(rate);
    currentSpeed = mSpeed; //remember what speed we are at. 
  }
}

speed_t ESC_30A::get_speed()
{
  return currentSpeed;
}

void ESC_30A::full_speed()
{
  throttle(ESC_30A_MAX_SPEED);
}

void ESC_30A::full_stop()
{
  throttle(ESC_30A_MIN_SPEED);
}


uint8_t ESC_30A::get_pin()
{
  return pin;
}

//allows the user to have direct control over the length of the pulse. 
void ESC_30A::throttle_pulse(uint16_t pulseWidth)
{
  bldc.writeMicroseconds(pulseWidth); 
}

//calibrates the controller
void ESC_30A::calibrate()
{  
  //Assuming that this program is starting just as the BLDC motor is turned on. 
  throttle(ESC_30A_MAX_SPEED); //before plugging in the batter of ESC set to full speed.

  #ifdef ESC_MSG
    Serial.println("Startup Sequence wait...");
  #endif
  delay(STARTUP_TIME);
  
  #ifdef ESC_MSG
    Serial.println("Should be in fast mode");
    Serial.println("Delaying");
  #endif
  delay(CALIBRATE_FAST_WAIT_TIME);
  
  throttle(ESC_30A_MIN_SPEED);
  #ifdef ESC_MSG
    Serial.println("Should be in low mode");
    Serial.println("Delaying");
  #endif
  delay(CALIBRATE_SLOW_WAIT_TIME);

  #ifdef ESC_MSG
    Serial.println("ESC_30A Calibrated!");
  #endif
    
}

void ESC_30A::wait_startup()
{
  delay(STARTUP_TIME); //just wait. Nothing else. 
}
