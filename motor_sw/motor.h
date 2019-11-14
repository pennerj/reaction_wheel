#ifndef REACTION_WHEEL_MOTOR
#define REACTION_WHEEL_MOTOR

#include <stdint.h>

class Motor
{
	public:
		Motor();
		
		/*
		The main interface to control the motor. Just set the speed. 
		*/
		virtual bool SetSpeed(uint8_t desiredSpeed) = 0;
		
		/*
		A getter function to see what the motor is supposed to be running at. 
		*/
		virtual uint8_t GetSpeed() = 0;
	protected:
		uint8_t targetSpeed;
		
	
};



#endif