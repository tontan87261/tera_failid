#ifndef Steering_encoder_h
#define Steering_encoder_h
#include "Arduino.h"

class Steering_encoder_data 
{
  public:
    Steering_encoder_data(int STEERING_ENCODER_CS_OUT_Pin, int STEERING_ENCODER_CLK_OUT_Pin, int STEERING_ENCODER_DO_IN_Pin); 
    int readEncoder();

  private:
    int _STEERING_ENCODER_CS_OUT_Pin;
    int _STEERING_ENCODER_CLK_OUT_Pin;
    int _STEERING_ENCODER_DO_IN_Pin;

};

#endif