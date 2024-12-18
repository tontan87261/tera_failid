#include "Autonomouscontrol.h"
#include "Arduino.h"

AutoControl::AutoControl(){
  RX_motors = 0;
  RX_servo = 0;
}

void AutoControl::readWriteSerial(int write_value){
  if (Serial.available() >= 2 * sizeof(int)) {
    for (int i = 0; i < 2; i++) {
      read_values[i] = Serial.read() | (Serial.read() << 8);
    }
    RX_servo = read_values[0];
    RX_motors = read_values[1];
    
    write_value = RX_servo; 
    Serial.write((byte)(write_value & 0xFF));
    Serial.write((byte)((write_value >> 8) & 0xFF));
  }
}

int AutoControl::getMotorsRX(){
  return RX_motors;
}

int AutoControl::getServoRX(){
  return RX_servo;
}
