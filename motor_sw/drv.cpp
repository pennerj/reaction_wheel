#include "drv.h"



DRV::DRV()
{
	//target speed is set in the motor class
}
		
/*
The main interface to control the motor. Just set the speed. 
*/
bool DRV::SetSpeed(uint8_t desiredSpeed)
{
	
	this->targetSpeed = desiredSpeed; //set what the speed is supposed to be. 
	
	//todo: do all of the other sutff to actually do stuff.....
	
	return false;
}

/*
A getter function to see what the motor is supposed to be running at. 
*/
uint8_t DRV::GetSpeed()
{
	return this->targetSpeed;
}