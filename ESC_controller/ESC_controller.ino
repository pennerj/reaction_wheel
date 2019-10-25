
/*
 * Created February 2, 2019 by Wesley Stirk
 * 
 * Created for the purpose of using the ESC_30A class to control a bldc motor
 * Assumes that there is a serial connection to the arduino that commands can be sent over. 
 * Kind of like a terminal to control the motor. 
 * 
 * Last updated February 15, 2019 by Wesley Stirk
 */

#include "ESC_30A.h"

#define CONTROL_PIN 9
#define BAUD_RATE 9600

//#define USE_CONTROL //comment out to use another sketch. Put in to use this one.
#ifdef USE_CONTROL //a bit of a hack to allow multiple sketches in the same project. 


 ESC_30A motor(CONTROL_PIN);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  motor.init();

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Ready!");
  while (!Serial.available()) {}; //wait for something on the serial line before beginning

  String comm = Serial.readString(); //get a command from the computer
  comm.remove(comm.length()-1); //remove the trailing \n from the string
  
  //start parsing the command
  if(comm.equals("full"))  //set it to full throttle
  {
    motor.full_speed(); 
  }
  else if (comm.equals("stop")) //stop the motor
  {
    motor.full_stop();
  }
  else if (isStringNumeric(comm) ) //determine if is a numnber
  {
    motor.throttle(comm.toInt()); //set the motor to that number (if appropriate)
  }
  else
  {
    Serial.println("Error input");
    Serial.print("Input: ");
    Serial.println(comm);
  }
}

/*
 * Accepts a string
 * Determines if all of the characters in the string are numeric
 */
bool isStringNumeric(String msg)
{
  for(int i = 0; i < msg.length(); ++i)
  {
    if(!isDigit(msg[i]))
    {
      return false;
    }
  }

  return true;
}

#endif
