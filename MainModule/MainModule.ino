#include "arduinoFFT.h"
 
#define SAMPLES 128           //Must be a power of 2
#define SAMPLING_FREQUENCY 2000 //Hz, must be less than 10000 due to ADC

#define E2 82.41
#define A2 110.60  
#define D3 146.9         //146.83
#define G3 199         //196.00
#define B3 249.6    //246.94
#define E4 332         //329.63

#define A B01010000
#define B B00010011
#define C B00111001
#define D B00010110
#define E B00110001
#define F B01110001
#define G B00011001


arduinoFFT FFT = arduinoFFT();
 
unsigned int sampling_period_us;
unsigned long microseconds;
 
double vReal[SAMPLES];
double vImag[SAMPLES];

double peak;
double desired_peak;
double precision = 0.2;
int treshold = 200;

//PID
double e;

int program_state = 0;
int adc_iterator = 0;


int ledPinLeft = 5;
int ledPinCenter = 4;
int ledPinRight = 3;

int display_counter = 0;

void setup() {
    Serial.begin(9600);
 
    sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
     cli();//diable interrupts
  DDRD = DDRD | B11111100;
  PORTD = PORTD | B11000000;

  
  DDRB = DDRB | B11111111;
  PORTB = PORTB | B11111111;

  
  
  //set up continuous sampling of analog pin 0

  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);
  TIMSK0 |= (1 << OCIE0A);
  
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;

  ADMUX |= (1 << REFS0); //set reference voltage
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //set ADC clock with 128 prescaler- 16mHz/128=125kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements

  ADCSRB |= (1 << ADTS1);
  ADCSRB |= (1 << ADTS0); //Set timer 0 to trigger adc

  sei();//enable interrupts
}




double myFFT(){
  /*FFT*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    //peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  return peakFind();
/*
  //Peak find  
  double maxY = 0;
  uint16_t IndexOfMaxY = 0;
  for (uint16_t i = 4; i < ((SAMPLES >> 1) + 1); i++) {
    if ((vReal[i-1] < vReal[i]) && (vReal[i] > vReal[i+1])) {
      if (vReal[i] > 200) {
        maxY = vReal[i];
        IndexOfMaxY = i;
        break;
      }
    }
  }

  if(IndexOfMaxY !=0){
  double delta = ((- vReal[IndexOfMaxY-1] + vReal[IndexOfMaxY+1]) / (vReal[IndexOfMaxY-1] +  vReal[IndexOfMaxY] + vReal[IndexOfMaxY+1]));
 double interpolatedX = ((IndexOfMaxY + delta)  * SAMPLING_FREQUENCY) / (SAMPLES-1);

  if(IndexOfMaxY==(SAMPLES >> 1)) //To improve calculation on edge values
    interpolatedX = ((IndexOfMaxY + delta)  * SAMPLING_FREQUENCY) / (SAMPLES);
  return interpolatedX;
  }
  else
  {
    return 0.0;
  }*/
}


double peakFind(){
  for(int i = 0; i< ((SAMPLES >> 1) + 1); i++)
    vImag[i] = vReal[i];
  for(int i = 0; i*2< ((SAMPLES >> 1) + 1); i++)
    vImag[i] = vImag[i] * vReal[i*2];
  for(int i = 0; i*3< ((SAMPLES >> 1) + 1); i++)
    vImag[i] = vImag[i] * vReal[i*3];

   //Peak find  
  double maxY = 0;
  uint16_t IndexOfMaxY = 0;
  for (uint16_t i = 4; i < ((SAMPLES >> 1) + 1); i++) {
    if ((vImag[i-1] < vImag[i]) && (vImag[i] > vImag[i+1])) {
      if (vImag[i] > maxY) {
        maxY = vImag[i];
        IndexOfMaxY = i;
        //break;
      }
    }
  }

  if(IndexOfMaxY !=0 && vReal[IndexOfMaxY] > treshold){
  double delta = ((- vReal[IndexOfMaxY-1] + vReal[IndexOfMaxY+1]) / (vReal[IndexOfMaxY-1] +  vReal[IndexOfMaxY] + vReal[IndexOfMaxY+1]));
 double interpolatedX = ((IndexOfMaxY + delta)  * SAMPLING_FREQUENCY) / (SAMPLES-1);

  if(IndexOfMaxY==(SAMPLES >> 1)) //To improve calculation on edge values
    interpolatedX = ((IndexOfMaxY + delta)  * SAMPLING_FREQUENCY) / (SAMPLES);
  return interpolatedX;
  }
  else
  {
    return 0.0;
  }
        
}

double frequencyCheck(double freq){
  if(freq>75.0 && freq <90.0){
        sevenSegment(E);
        precision = 0.3;
       return E2;   
  }
  if(freq>90.0 && freq <125.0){
    sevenSegment(A);
    precision = 0.4;
    return A2;
  }
  if(freq>125.0 && freq <165.0){
    sevenSegment(D);
    precision = 0.5;
    return D3;
  }
  if(freq>165.0 && freq <215.0){
    sevenSegment(G);
    precision = 0.7;
    return G3;
  }
  if(freq>215.0 && freq <275.0){
    sevenSegment(B);
    precision = 0.8;
    return B3;
  }
  if(freq>275.0){
    sevenSegment(E);
    precision = 0.9;
    return E4;   
  }
}




void flashLed(double diff){
  if(abs(diff) < precision){
    digitalWrite(ledPinCenter, 1);
    analogWrite(ledPinLeft, 0);
    analogWrite(ledPinRight, 0);
    return;
  }
  if(diff > 0){
    digitalWrite(ledPinCenter, 0);
    analogWrite(ledPinLeft, 255);
    analogWrite(ledPinRight, 0);
    return;
  }
  
  if(diff < 0){
    digitalWrite(ledPinCenter, 0);
    analogWrite(ledPinLeft, 0);
    analogWrite(ledPinRight, 255);
    return;
  }
}

void sevenSegment(char letter){
  digitalWrite(13, letter & B10000000);
  digitalWrite(12, letter & B01000000);
  digitalWrite(11, letter & B00100000);
  digitalWrite(10, letter & B00010000);
  digitalWrite(9, letter & B00001000);
  digitalWrite(8, letter & B00000100);
  digitalWrite(7, letter & B00000010);
  digitalWrite(6, letter & B00000001);
}

void displayClear(){
    PORTD = PORTD | B11000000;
    PORTB = PORTB | B11111111;
    digitalWrite(ledPinCenter, 0);
    analogWrite(ledPinLeft, 0);
    analogWrite(ledPinRight, 0);
}
 
void loop() {
  if (program_state == 2) {
    peak = myFFT();
    if(peak != 0.0){
      desired_peak = frequencyCheck(peak);
      e = desired_peak - peak;
      flashLed(e);
      //Serial.println(peak);
      if(e>precision)
        Serial.write(50);
      else if(e<-precision)
        Serial.write(-50);
      else
        Serial.write(0);
    }
    program_state = 0;
    }
    
}

ISR(TIMER0_COMPA_vect) {};

ISR(ADC_vect) {//when new ADC value ready
  if (program_state == 0) {
    int test = ADC;
    if (test > 480 || test < 430) {
      program_state = 1;
      adc_iterator = 0;
      display_counter = 0;
    }else{
      if(display_counter++ > 500){
        displayClear();
        Serial.write(0);
        display_counter = 0;
      }
    }
  } else if (program_state == 1) {
     vReal[adc_iterator] = ADC;
     vImag[adc_iterator++] = 0;
    if (adc_iterator == SAMPLES) {
      adc_iterator = 0;
      program_state = 2;
    }
  }
}
