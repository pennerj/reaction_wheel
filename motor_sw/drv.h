#ifndef REACTION_WHEEL_DRV_MOTOR
#define REACTION_WHEEL_DRV_MOTOR

#include "motor.h"

class DRV : protected Motor
{
	public :
		DRV();
		
		/*
		The main interface to control the motor. Just set the speed. 
		*/
		bool SetSpeed(uint8_t desiredSpeed);
		
		/*
		A getter function to see what the motor is supposed to be running at. 
		*/
		uint8_t GetSpeed();
		
	private: 
		
	
	
};

#endif