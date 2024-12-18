#ifndef Receiver_h
#define Receiver_h
#include "Arduino.h"

class ReceiverData 
{
  public:
    ReceiverData();
    long readChannelRaw(byte channelInput);
    long readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue);
    long readSwitch(byte channelInput, int defaultValue);
    int printChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue);
  private:
    int _channelInput;

};

#endif
