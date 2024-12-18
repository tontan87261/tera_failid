#include "Steering.h"
#include "Arduino.h"
#define DIR 42      //suuna määramise pin
#define PUL 40      //steering pulse pin
#define STEERING_CYCLE 15 //keeramise kiirus
#define DELAY 100   //OLI #define DELAY 100
#define DEADZONE 10
#define WINDOW_SIZE 10

int INDEX = 0;
int VALUE = 0;
int SUM = 0;
int READINGS[WINDOW_SIZE];
int ENCODER1 = 0;

/*
int mapping(int a1, int a2, int b1, int b2, int input){
  int inValNorm = input - a1;
  int aUpperNorm = a2 - a1;
  int normPos = inValNorm / aUpperNorm;

  int bUpperNorm = b2 - b1;
  int bValNorm = normPos * bUpperNorm;
  int outVal = b1 + bValNorm;

  return outVal;
}
*/

Steering::Steering()
{
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
}
//value1 = steering, value5 =RC/TO switch,wheel = teleop steering value
 int Steering::Left_Right(int value1, int value5, int encoder, int wheel)
{
 encoder = encoder / 10;

  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  VALUE = encoder;                   // Read the next sensor value
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

  ENCODER1 = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result


//-----------------------TELEOP KEERAMINE------------------------------------------------------------------------------------------------------------------
//Steering range = 0(vasak) - 655
//Steering centre = 328
//Encoder values = 365 - 170 (195)

if(value5 == 1){
    //encoder = encoder / 10;
    //Serial.print(encoder);
    //Serial.print("\n");
    int realwheel = 268;
    
    realwheel = map(wheel, 0, 655, 170, 365);
    
      
    //mapping(0, 655, 170, 365, wheel);
    //Serial.println(realwheel);
    int difference = ENCODER1 - realwheel;
    //Serial.println(difference);

    if(difference < 0 &&  abs(difference) > DEADZONE){
        digitalWrite(DIR, HIGH);   //PAREMALE
  
        for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
          //these 4 lines result in 1 step:
          digitalWrite(PUL, HIGH); //PIN 6 = stepPin
          delayMicroseconds(DELAY);
          digitalWrite(PUL, LOW);
          delayMicroseconds(DELAY);
        }
      }
      if(difference > 0 && abs(difference) > DEADZONE){
        digitalWrite(DIR, LOW);   //VASAKULE
  
        for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
          //these 4 lines result in 1 step:
          digitalWrite(PUL, HIGH); //PIN 6 = stepPin
          delayMicroseconds(DELAY);
          digitalWrite(PUL, LOW);
          delayMicroseconds(DELAY);
        }
      }
    
  }

/*
    if(digitalRead(TO_RIGHT) == 1){
      digitalWrite(DIR, LOW);   //PAREMALE

      for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
        //these 4 lines result in 1 step:
        digitalWrite(PUL, HIGH); //PIN 6 = stepPin
        delayMicroseconds(DELAY);
        digitalWrite(PUL, LOW);
        delayMicroseconds(DELAY);
      }
    }
    if(digitalRead(TO_LEFT) == 1){
      digitalWrite(DIR, HIGH);   //VASAKULE

      for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
        //these 4 lines result in 1 step:
        digitalWrite(PUL, HIGH); //PIN 6 = stepPin
        delayMicroseconds(DELAY);
        digitalWrite(PUL, LOW);
        delayMicroseconds(DELAY);
      }
    }
  }
*/
//----------------------------RC KEERAMINE----------------------------------------------------------------
//Vahemik = 365-170
  else if(value5 == 0)
  {
    
    //encoder = encoder / 10;
    
    //int difference = encoder - value1;
    int difference = ENCODER1 - value1;
    //Serial.println(encoder);

    if(difference < 0 &&  abs(difference) > DEADZONE){
      digitalWrite(DIR, HIGH);   //PAREMALE

      for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
        //these 4 lines result in 1 step:
        digitalWrite(PUL, HIGH); //PIN 6 = stepPin
        delayMicroseconds(DELAY);
        digitalWrite(PUL, LOW);
        delayMicroseconds(DELAY);
      }
    }
    if(difference > 0 && abs(difference) > DEADZONE){
      digitalWrite(DIR, LOW);   //VASAKULE

      for (int i = 0; i < STEERING_CYCLE; i++){    //ühe tsükli pikkus, mida suurem seda pikem ring
        //these 4 lines result in 1 step:
        digitalWrite(PUL, HIGH); //PIN 6 = stepPin
        delayMicroseconds(DELAY);
        digitalWrite(PUL, LOW);
        delayMicroseconds(DELAY);
      }
    }
  }
}
