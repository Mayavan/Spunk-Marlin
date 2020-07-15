/*
    PololuDriver.h - Library for DRV8825 Stepper Driver
*/

#ifndef _POLOLUDRIVER_h
#define _POLOLUDRIVER_h

#include "Arduino.h"

class PololuStepper
{
	public:
		PololuStepper(int _stepPin, int _dirPin, int _enablePin); //Constructor

		void setDir(short _dir);	//Bepaal Draairichting

		bool stepOn();				//Step Handmatig
									//(stepOff moet daarna komen)

		void stepOff();				//Concludeer Stap handmatig
									//(Moet na StepOn komen)

		//Beweeg een aantal stappen. (aantal kan negatief zijn)
		void autoStep(long amount, int _delay);

		//Beweeg naar doelpositie met gegegeven delay (in microseconden)
		void moveTo(long targetPos, int _delay);

		long position = 0;			//Positie

		void enable();

	private:
		short dir = 1;
		bool stepped = false;

		int stepPin;
		int dirPin;
		int enablePin;

};

#endif
