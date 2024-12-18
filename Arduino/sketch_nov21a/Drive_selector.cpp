#include "Drive_selector.h"
#include "Arduino.h"

Drive_selector_switch::Drive_selector_switch()
{
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
}

//value 3 = RC throttle, value 6 = FWD/AWD/RWD, throttle = TO throttle, value5 = TO/RC switch, value8 = e turn off, ebrake = lidar info(brake)
int Drive_selector_switch::Drive_mode(int value3, int value6, int throttle, int value5, int value8, int ebrake)
{
  //TO mode
  if(ebrake == 0){
    
    if(value5 == 1){
        if(value6 == 1) //Honda
        {
          if(throttle > 0)
          {
            analogWrite(8, throttle);
            analogWrite(9, 0);
          }
          
          if(throttle < 0)
          {
            analogWrite(9, 255);
            analogWrite(8, abs(throttle));
          }
         
          return 1;
        }
      }
    
          
      else if(value6 == 2) //Audi
      {
        analogWrite(7, value3);
        analogWrite(8, value3);
        return 2;
      }
      else if(value6 == 3) //BMW
      {
        analogWrite(7, value3);
        return 3;
      }
       
  }

  //RC mode
  if(ebrake == 0){
    if(value5 == 0){
        if(value6 == 1) //Honda
        {
          if(value3 > 0)
          {
            analogWrite(8, value3);
            analogWrite(9, 0);
          }
          
          if(value3 < 0)
          {
            analogWrite(9, 255);
            analogWrite(8, abs(value3));
          }
         
          return 1;
        }
      }

    }
      else if(value6 == 2) //Audi
      {
        analogWrite(7, value3);
        analogWrite(8, value3);
        return 2;
      }
      else if(value6 == 3) //BMW
      {
        analogWrite(7, value3);
        return 3;
      }
       
  }
