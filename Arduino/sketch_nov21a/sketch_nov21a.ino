#include <Stepper.h>
#include <IBusBM.h>

#include "Receiver.h"
#include "Steering_encoder.h"
#include "Drive_selector.h"
#include "Steering.h"
#include "Autonomouscontrol.h"

ReceiverData receiver;
Steering_encoder_data encoder(30, 28, 32);
Drive_selector_switch drive_select;
Steering steering;
AutoControl autonomous;

// Struct for sending data
struct sendata {
  volatile int val;
};

//Struct for receiving data
struct receive { 
  volatile int x;
  volatile int y;
};
struct sendata values;
struct receive bytes;
const int total_bytes=2*sizeof(int);
int i;
byte buf[total_bytes];

int wheel = 267;
int moving = 0;
int32_t ebrake = 0;



void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

}

void loop()
{
  
  
  
  int value1 = 267;
  int value5 = 1;
  //Read info from receiver-------------------------------
  value1 = receiver.readChannel(0, 365, 170, 0);        //Steering
  int value2 = receiver.readChannel(1, 255, -255, 0);   
  int value3 = receiver.readChannel(2, 255, -255, 0);   //RC Throttle
  int value4 = receiver.readChannel(3, 255, -255, 0);
  value5 = receiver.readSwitch(4, 1);                   //RC/TO Switch
  int value6 = receiver.readChannel(5, 1, 3, 2);        //FWD/AWD/RWD
  int value8 = receiver.readSwitch(7, 1);               //KILLSWITCH
  // Reciver data print
  //Serial.print(receiver.readChannelRaw(0));
  //Serial.println(value5);
  //Serial.print("\n");




  //Check if Killswitch is off 
  
  if (value8 == 1){
    //INFO FROM JETSON
    autonomous.readWriteSerial(21);
    wheel = autonomous.getServoRX();
    moving = autonomous.getMotorsRX();
    /*
    if (Serial.available() >= total_bytes){
      i = 0;
      while(i < total_bytes){
        buf[i] = Serial.read();
        i++;
      }
    
      memmove(&bytes,buf,sizeof(bytes));
      //int val = int(trunc(bytes.value));
      wheel = bytes.x;
      moving = bytes.y;
      values.val = wheel;
      Serial.write((const uint8_t*)&values, sizeof(values));
    }
    */
    //Drive selector data ---------------------------
  int Drive_number = drive_select.Drive_mode(value3, value6, moving, value5, value8, ebrake);
  //Serial.print(Drive_number);
  //Serial.println("\n");

  //Steering data ---------------------------
  int encoder_data = (encoder.readEncoder());
  steering.Left_Right(value1, value5, encoder_data , wheel);

  }
  else if(value8 == 0){
        analogWrite(8, 0);
        analogWrite(9, 0);
  }



}
