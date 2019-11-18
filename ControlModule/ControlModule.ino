/*
AUTHOR: Hazim Bitar (techbitar)
DATE: Aug 29, 2013
LICENSE: Public domain (use at your own risk)
CONTACT: techbitar at gmail dot com (techbitar.com)

*/


//#include <SoftwareSerial.h>

#include <Servo.h>
//SoftwareSerial BTSerial(10, 11); // RX | TX
Servo myservo;  // create servo object to control a servo
int pwm = 1500;   // for incoming serial data

int i = 0;
void setup() 
{
  Serial.begin(9600);
  //Serial.println("Enter AT commands:");
  //BTSerial.begin(9600);  // HC-05 default speed in AT command more
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
          myservo.writeMicroseconds(pwm-2*sig);
        if(sig>100)
          myservo.writeMicroseconds(pwm + 2*(256 - sig));
             
  }else{
    //myservo.write(pwm);
  }
  //delay(50);
 
  // Keep reading from Arduino Serial Monitor and send to HC-05
}
