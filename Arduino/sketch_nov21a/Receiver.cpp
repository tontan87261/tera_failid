#include "Receiver.h"
#include "Arduino.h"
#include <IBusBM.h>

IBusBM ibus;

ReceiverData::ReceiverData()
{
  ibus.begin(Serial1);
}


long ReceiverData::readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) 
{
  uint16_t ch = ibus.readChannel(channelInput);
  if (1550 > ch > 1450) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

long ReceiverData::readSwitch(byte channelInput, int defaultValue) 
{
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, defaultValue);
  return (ch > 50);
}

int ReceiverData::printChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue){
  int value = readChannel(channelInput, minLimit, maxLimit, defaultValue);
  //Serial.println(value);
  //Serial.print("\n");
}
