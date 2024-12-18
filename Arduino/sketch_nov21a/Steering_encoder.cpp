#include "Steering_encoder.h"
#include "Arduino.h"
#include <Stepper.h>

Steering_encoder_data::Steering_encoder_data(int STEERING_ENCODER_CS_OUT_Pin, int STEERING_ENCODER_CLK_OUT_Pin, int STEERING_ENCODER_DO_IN_Pin) 
{
  pinMode(STEERING_ENCODER_CS_OUT_Pin, OUTPUT);
  pinMode(STEERING_ENCODER_CLK_OUT_Pin, OUTPUT);
  pinMode(STEERING_ENCODER_DO_IN_Pin, INPUT);
  _STEERING_ENCODER_CS_OUT_Pin = STEERING_ENCODER_CS_OUT_Pin; // Replace with your pin number
  _STEERING_ENCODER_CLK_OUT_Pin = STEERING_ENCODER_CLK_OUT_Pin; // Replace with your pin number
  _STEERING_ENCODER_DO_IN_Pin = STEERING_ENCODER_DO_IN_Pin;  // Replace with your pin number
}




int Steering_encoder_data::readEncoder() 
  {
    int i;
    uint8_t Resolution = 12;
    uint32_t bitstart = 0x0800;
    uint16_t encoder_reading = 0;
    digitalWrite(_STEERING_ENCODER_CS_OUT_Pin, LOW);
    digitalWrite(_STEERING_ENCODER_CLK_OUT_Pin, LOW);
    for (i = (Resolution - 1); i >= 0; i--) {
      digitalWrite(_STEERING_ENCODER_CLK_OUT_Pin, HIGH);
      if (digitalRead(_STEERING_ENCODER_DO_IN_Pin))
        encoder_reading |= bitstart;
      digitalWrite(_STEERING_ENCODER_CLK_OUT_Pin, LOW);
      bitstart = bitstart >> 1;
      if (i == 0) {
        digitalWrite(_STEERING_ENCODER_CLK_OUT_Pin, HIGH);
        if (digitalRead(_STEERING_ENCODER_DO_IN_Pin))
          encoder_reading |= bitstart;
      }
    }
    digitalWrite(_STEERING_ENCODER_CS_OUT_Pin, HIGH);
    //Serial.println(encoder_reading);
    return encoder_reading;
  }
