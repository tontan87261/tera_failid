#ifndef Autonomouscontrol_h
#define Autonomouscontrol_h
#include "Arduino.h"

class AutoControl {
  public:
    AutoControl();

    int read_values[2];
    int RX_motors;
    int RX_servo;


    void readWriteSerial(int write_value);
    int getMotorsRX();
    int getServoRX();
    
};

#endif
