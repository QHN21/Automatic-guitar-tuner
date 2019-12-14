/*
AUTHOR: Marcin Kuchenbecker
DATE: 13.12.2019
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
int pwm = 1500;   // pwm signal for stop

int reset_counter = 0;

void setup() 
{
  Serial.begin(9600);
  myservo.attach(3);  // attaches the servo on pin 9 to the servo object
  myservo.writeMicroseconds(pwm);
}

void loop()
{
  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if(Serial.available()){
        int sig = Serial.read();
        //Serial.println(sig);
        if(sig< 101)
          myservo.writeMicroseconds(pwm-sig);
        if(sig>100)
          myservo.writeMicroseconds(pwm + (256 - sig));   
         reset_counter = 0;        
  }else{
    if(reset_counter++ > 500){
      myservo.writeMicroseconds(pwm);
      reset_counter = 0;
    }
  }
  delay(1);
}
