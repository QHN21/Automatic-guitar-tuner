
#include "SoftwareSerial.h"
#define SAMPLES 128           //Must be a power of 2
#define POWER 7
#define SAMPLING_FREQUENCY 2000 //Hz, must be less than 10000 due to ADC

#define MYPI    3.14159265359

#define E2 82.41
#define A2 110.60  
#define D3 146.83         //146.83
#define G3 197.80  //199         //196.00
#define B3  246.94  //249.6    //246.94
#define E4   329.63 //  332         //329.63

#define A B01010000
#define B B00010011
#define D B00010110
#define E B00110001
#define G B00011001

 
double vReal[SAMPLES];
double vImag[SAMPLES];

double peak;
double desired_peak;
double precision = 0.2;
int treshold = 200;

//PID
double K = 4.0;
double Tp = 0.2;
double Ti = 50.0;
double Td = 0.0;

double u = 0.0;
double u_past = 0.0;

double r[3] = {0.0, 0.0, 0.0};
double e[3] = {0.0, 0.0, 0.0};
//PID END


int program_state = 0;
int adc_iterator = 0;


int ledPinLeft = 5;
int ledPinCenter = 4;
int ledPinRight = 3;

int display_counter = 0;
volatile int time_counter = 0;

// /*Debugging
SoftwareSerial mySerial(A4, 2);
//Debugging end*/

void setup() {
  Serial.begin(9600);
  
  mySerial.begin(9600);
  
  
  cli();//diable interrupts
  
  DDRD = DDRD | B11111100;
  PORTD = PORTD | B11000000;

  
  DDRB = DDRB | B11111111;
  PORTB = PORTB | B11111111;

  //PID

  r[0] = K*(1 + Tp/(2*Ti)+Td/Tp);
  r[1] = K*(Tp/(2*Ti)-2*Td/Tp-1);
  r[2] = K*Td/Tp;
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
  HammingWindowing();
  ComputeFFT();
  ComputeMagnitude();
  return peakFind();
}

void ComputeFFT(){

  //BIT REVERSE
  uint16_t j = 0;
  for (uint16_t i = 0; i < (SAMPLES - 1); i++) {
    if (i < j) {
      double tmp;
      tmp = vReal[i];
      vReal[i] = vReal[j];
      vReal[j] = tmp;
    }
    uint16_t k = (SAMPLES >> 1);
    while (k <= j) {
      j -= k;
      k >>= 1;
    }
    j += k;
  }

  //FFT
  
  uint16_t l2 = 1;

  for (uint8_t l = 0; (l < POWER); l++) {
    uint16_t l1 = l2;
    l2 <<= 1;
    uint16_t a = 0;
    
    
    double u1 = 1.0; //cos
    double u2 = 0.0;  //sin

    for (j = 0; j < l1; j++) {
        u1 = cos(-2*MYPI*a/SAMPLES);
        u2 = sin(-2*MYPI*a/SAMPLES);

        a += 1 << (POWER - l - 1);

       for (uint16_t i = j; i < SAMPLES; i += l2) {
          uint16_t i1 = i + l1;
          double t1 = u1 * vReal[i1] - u2 * vImag[i1];
          double t2 = u1 * vImag[i1] + u2 * vReal[i1];
          vReal[i1] = vReal[i] - t1;
          vImag[i1] = vImag[i] - t2;
          vReal[i] += t1;
          vImag[i] += t2;
       }
    }
  }
}

void ComputeMagnitude(){
  for (uint16_t i = 0; i <(SAMPLES >> 1) + 1; i++) {
    vReal[i] = sqrt(vReal[i]*vReal[i] + vImag[i]*vImag[i]);
  }
}

void HammingWindowing(){
  double samplesMinusOne = (double(SAMPLES) - 1.0);
  for (uint16_t i = 0; i < (SAMPLES >> 1); i++) {
    double ratio = (double(i) / samplesMinusOne);
    double  weighingFactor = 0.54 - (0.46 * cos(2 * MYPI * ratio));
    vReal[i] *= weighingFactor;
    vReal[SAMPLES - (i + 1)] *= weighingFactor;
  }
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
    double interpolatedX = ((IndexOfMaxY + delta)  * SAMPLING_FREQUENCY) / (SAMPLES);
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
    precision = 0.2;
    return E2;   
  }
  if(freq>90.0 && freq <125.0){
    sevenSegment(A);
    precision = 0.2;
    return A2;
  }
  if(freq>125.0 && freq <165.0){
    sevenSegment(D);
    precision = 0.3;
    return D3;
  }
  if(freq>165.0 && freq <215.0){
    sevenSegment(G);
    precision = 0.8;
    return G3;
  }
  if(freq>215.0 && freq <275.0){
    sevenSegment(B);
    precision = 0.5;
    return B3;
  }
  if(freq>275.0){
    sevenSegment(E);
    precision = 0.5;
    return E4;   
  }
}

int pid(double e_current){

  //PID
  e[2] = e[1];
  e[1] = e[0];
  e[0] = e_current;
  u_past = u;
  u = u_past + r[0]*e[0] + r[1]*e[1] + r[2]*e[2];

  //Ograniczenia
  if(u< 10 && u > 0) u = 10;
  if(u < 0 && u > -10) u = -10;
  if(u>60) u = 60;
  if(u < -60)u = -60;
  if(abs(e_current)<precision) u = 0;
  
  return int(u);
}


void flashLed(double diff){
  if(abs(diff) < precision){
    digitalWrite(ledPinCenter, 1);
    digitalWrite(ledPinLeft, 0);
    digitalWrite(ledPinRight, 0);
    return;
  }
  if(diff > 0){
    digitalWrite(ledPinCenter, 0);
    digitalWrite(ledPinLeft, 1);
    digitalWrite(ledPinRight, 0);
    return;
  }
  
  if(diff < 0){
    digitalWrite(ledPinCenter, 0);
    digitalWrite(ledPinLeft, 0);
    digitalWrite(ledPinRight, 1);
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
    digitalWrite(ledPinLeft, 0);
    digitalWrite(ledPinRight, 0);
}

void pidReset(){
  Serial.write(0);
  e[0] = 0;
  e[1] = 0;
  e[2] = 0;
  u = 0.0;
  u_past = 0.0;
  return;
}
 
void loop() {
  if (program_state == 2) {
    peak = myFFT();
    if(peak != 0.0){
      desired_peak = frequencyCheck(peak);
      //double e_current = 1200*log(desired_peak/peak)/log(2)
      double e_current = desired_peak - peak;
      int u_current = pid(e_current);
      flashLed(e_current);

      //Debugging
      char tmp[25];
      sprintf(tmp, "%u %d %u ", uint16_t(peak*100),  u_current, uint16_t(desired_peak*100));      
      mySerial.println(tmp);
      //Debugging end

      
      while(time_counter < int(Tp*1000)*2);
      time_counter = 0;
      Serial.write(u_current); 

      
      adc_iterator = 0;
      display_counter = 0;
 
      program_state = 1;
    }else{
      displayClear();
      pidReset();
      program_state = 0;
    }
  
    }
    
}

ISR(TIMER0_COMPA_vect) {};

ISR(ADC_vect) {//when new ADC value ready
  if (program_state == 0) {
    int test = ADC;
    if (test > 440 || test < 380) {
      program_state = 1;
      adc_iterator = 0;
      display_counter = 0;
    }else{
      if(display_counter++ > 500){
        displayClear();
        pidReset();
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
    time_counter++;
  } else if(program_state == 2 ) {
    time_counter++;
  }
}
