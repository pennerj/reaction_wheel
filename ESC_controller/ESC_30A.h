/*
 * Created on February 2, 2019 by Wesley Stirk
 * 
 * This class has the basic members and functions needed to control the 30A BLDC ESC from Hobbypower
 * It relies heavily on the Servo library for much of its functionality. 
 * 
 * Last Updated February 27, 2019 by Wesley Stirk
 */

#ifndef BLDC_ESC_30A
#define BLDC_ESC_30A

#include "Arduino.h"
#include <stdint.h>
#include <Servo.h>

typedef uint8_t speed_t; //used for recording the speed

//defines for use by others

#define ESC_DEBUG //uncomment to see debug messages
#define ESC_MSG //uncomment to see messages

#define ESC_30A_MAX_SPEED 100 
#define ESC_30A_MIN_SPEED 0

#define PRE_CALIBRATE_LOW 1225 //found using experimentation. pulse width
      // a speed that the controller should recognize as viable 0 speed.

//TODO: make sure this is an accurate time. 
#define STARTUP_TIME 2000 //given in milliseconds. 
//Approximately how long it takes for the controller to go through various startup procedures. 
//It should emit various beeps during this time. A special tone after a self test.
//Then a battery count. Then it should be ready for whatever comes next. 

#define CALIBRATE_FAST_WAIT_TIME  3000 //given in milliseconds
//After calibration has moved through the initial startup it takes time
//to register the new full speed. 

 #define CALIBRATE_SLOW_WAIT_TIME 1200 //given in milliseconds
 //After registering the full speed in calibration. Wait at least 1 second before
 //the esc has registered the slowest speed. 

//don't touch defines
#define IMPOSSIBLE_PIN -1



class ESC_30A
{
  public:
    //constructor of class
    //initializes the controller with the controlPin
    ESC_30A(uint8_t controlPin);

    /*
     * Goes through the neede sequence to calibrate the ESC to a standard
     * pulse width. Should work pretty well. 
     */
    void calibrate();

    /*
     * Controls the speed of the motor. The function that will be mainly used. 
     * speed needs to be a value between 0 and 100 representing the percent of maximum
     */
    void throttle(speed_t mSpeed);
  
    /*
     * The init function initializes everything before the controller is used. 
     * Must be called inside of a function.
     */
    void init(int8_t newPin = IMPOSSIBLE_PIN);

    /*
     * returns the current speed of the motor. 
     */
    speed_t get_speed();

    /*
     * Sets the motor to full speed
     */
    void full_speed();

    /*
     * Turns the motor all the way off.  
     */
    void full_stop();

    /*
     * returns the saved value of the pin
     */
     uint8_t get_pin();

     /*
      * Directly controls the length of a pulse so that you can have more control
      * over the pulse width
      */
      void throttle_pulse(uint16_t pulseWidth);

      /*
       * Causes a delay for the proper amount of time to allow the ESC to go through it's start up routine. 
       * Useful for when programs will be starting automatically, without human input. 
       */
      void wait_startup();

  private:
    speed_t currentSpeed; //keeps track of the current speed of the motor
    uint8_t pin;
    Servo bldc; //the Servo library that will do a lot of the controls. 
  
};


#endif
