/*
  Design and created by Blascarr

  TFTCanvas
  Name    : Blascarr
  Description: controlServo.h
  version : 1.0

	sensorServoControl is a library for control with analog sensors.

	The main objective is improve the performance and mechanical response 
	depending on values associated with analog inputs.

	Save up working operation energy performance.
	Compute time event intervals to read input signal
	Compute buffer based on mean values.
	Mapping in different range values.
	
  	
  	Blascarr invests time and resources providing this open source code like some other libraries, please
  	respect the job and support open-source software.
    
    Written by Adrian for Blascarr
*/

#ifndef sensorservocontrol_h
#define sensorservocontrol_h	
	#if defined(ARDUINO) && ARDUINO >= 100
		#include "Arduino.h"
	#else
		#include "WProgram.h"
	#endif
    #include "Servo.h"
    #include <ControlledServo.h>

    class SSC {
    	public:
        ControlledServo *_cServo;
    		int _sensorPin;
        int _PWMPin;
        int _sensorDiff = 5; //Difference in 5 units over 1023 minimize noise for reading values

        bool _interrupt = true;
        bool _stopDetach = true;
        bool _meanCompute = true;

        long _minVal=0;
        long _maxVal=180;

        unsigned long currentMillis,oldMillis;
        unsigned long refreshTime= 10; // 50ms just enough for Reading Values and non-blocking events when DEBUG is ON
        int _currentValue,currentSValue,oldSValue; 

    		SSC( ControlledServo &cServo,int PWMPIN, int input);
    		setPIN(int input);
        setPWMPIN(int PWMPIN);
        setrefresh( unsigned long time);
        setSensorDiff( int sensorDiff);
        setMap(int minVal, int maxVal);
        setStopDetach( bool stopDetach);
        setInterrupt(bool interrupt);

        timer();
        compute();
    		compute(void (*function)(void), void (*funcNotMoving)(void)=NULL );
        int computeSensor();
    		int mapValue(int value);
    		mean();

    };


#endif