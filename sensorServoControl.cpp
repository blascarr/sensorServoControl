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

#include <sensorServoControl.h> 

#define debug 0 //Debug ON handicap real time for small refreshtime events over 10ms. For 50ms event, DEBUG can work, but is not recommended.

SSC::SSC( ControlledServo &cServo,int PWMPIN,  int input){
	SSC::_cServo = &cServo;
	SSC::setPIN(input);
  SSC::setPWMPIN(PWMPIN);
}

SSC::setPIN( int input){
  SSC::_sensorPin = input;
}

SSC::setPWMPIN(int PWMPIN){
  SSC::_PWMPin = PWMPIN;
}
SSC::setrefresh( unsigned long time){
  SSC::refreshTime = time;
}

SSC::setSensorDiff( int sensorDiff){
  SSC::_sensorDiff = sensorDiff;
}

SSC::setMap(int minVal, int maxVal){
  SSC::_minVal=minVal;
  SSC::_maxVal=maxVal;
}

SSC::setStopDetach( bool stopDetach){
  SSC::_stopDetach = stopDetach;
}

SSC::setInterrupt(bool interrupt){
  SSC::_interrupt = interrupt;
}

//-----------------------------------//
//This function must appear in a loop
//Compute execute just one action in Servo in response to the Analog reading
SSC::timer(){
  SSC::currentMillis = millis();
  if(SSC::currentMillis-SSC::oldMillis >= SSC::refreshTime){
    int value = SSC::computeSensor();
    
    if(  SSC::_currentValue != value  ){
      if (debug){Serial.print("Current Value : ");Serial.println(value);}
      if (debug){Serial.print("Map Value : ");Serial.println(SSC::mapValue(value));}
      
      _cServo->moveTo( SSC::mapValue(value) );

      SSC::_currentValue = value;
    }
    SSC::oldMillis  = SSC::currentMillis;
  }
}

SSC::compute(){
  SSC::timer();

  if (_interrupt){
    if(_cServo->moving()){

      _cServo->_servo->attach(_PWMPin);
      
      _cServo->update();
    }else{
      if(_stopDetach)
        _cServo->_servo->detach();
    }
  }else{
    while(_cServo->moving()){
      _cServo->update();
    }
    if(_stopDetach)
      _cServo->_servo->detach();
  }
}

//Compute can specified more actions with customized functions in response to the reading value
//We can specified a function when ControlledServo is moving and some other optional function when ControlledServo is not moving
SSC::compute(void (*funcMoving)(void),void (*funcNotMoving)(void)){
  
  SSC::timer();

  if(_cServo->moving()){
    _cServo->_servo->attach(_PWMPin);
    funcMoving();

  }else{
    if(_stopDetach)
      _cServo->_servo->detach();
    funcNotMoving();
  }
}


int SSC::computeSensor(){
  SSC::currentSValue = analogRead(SSC::_sensorPin);

  if ( abs(SSC::currentSValue-SSC::oldSValue) > SSC::_sensorDiff){
    SSC::oldSValue = SSC::currentSValue;
    //if (debug){Serial.print("Sensor Value Switch: ");Serial.println(SSC::currentSValue);};
    return SSC::currentSValue ;
  }
}


int SSC::mapValue(int value){
  return map(SSC::_currentValue,0,1023,SSC::_minVal,SSC::_maxVal);
}