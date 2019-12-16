
#include "SoftwareSerial.h"
#define SAMPLES 128           
#define POWER 7
#define SAMPLING_FREQUENCY 2000

#define MYPI    3.14159265359

#define E2  81.7         // 82.40
#define A2 110.00        //110.00
#define D3 145.83        //146.83
#define G3 197.70        //196.00
#define B3 247.8         //246.94
#define E4 329.42        //329.63

#define A B01010000
#define B B00010011
#define D B00010110
#define E B00110001
#define G B00011001


#define LED_PIN_LEFT    5
#define LED_PIN_CENTER  4
#define LED_PIN_RIGHT   3
#define FFT_AMPLITUDE_TRESHOLD    200
 

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

volatile double precision = 0.2;

double vReal[SAMPLES];
double vImag[SAMPLES];

int program_state = 0;
int adc_iterator = 0;
int display_counter = 0;
volatile int time_counter = 0;

// /*Debugging
SoftwareSerial mySerial(A4, 2);
//Debugging end*/

void setup() {
  Serial.begin(9600);
  
  mySerial.begin(9600);
  
  //PID
  
  r[0] = K*(1 + Tp/(2*Ti)+Td/Tp);
  r[1] = K*(Tp/(2*Ti)-2*Td/Tp-1);
  r[2] = K*Td/Tp;
  
  cli();  //diable interrupts

  //sets LED and seven segment diplay pins
  
  DDRD = DDRD | B11111100;
  PORTD = PORTD | B11000000;

  DDRB = DDRB | B11111111;
  PORTB = PORTB | B11111111;

  //set up for continuous sampling of analog pin 0

  TCCR0A = 0;   // set entire TCCR0A register to 0
  TCCR0B = 0;   // same for TCCR0B
  
  TCNT0  = 0;   //initialize counter value to 0
  
  OCR0A = 124;  //sets compare match register for 2khz increments: 124 = (16*10^6) / (2000*64) - 1 (must be <256)
  TCCR0A |= (1 << WGM01);              // turn on CTC mode 
  TCCR0B |= (1 << CS01) | (1 << CS00); // Sets CS01 and CS00 bits for 64 prescaler  
  TIMSK0 |= (1 << OCIE0A);
  
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;

  ADMUX |= (1 << REFS0); //set reference voltage
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //set ADC clock with 128 prescaler- 16mHz/128=125kHz
  ADCSRA |= (1 << ADATE); //enabbles auto trigger
  ADCSRA |= (1 << ADIE);  //enables interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  //enables ADC
  ADCSRA |= (1 << ADSC);  //start ADC measurements
  ADCSRB |= (1 << ADTS1); //Sets timer 0 to trigger adc
  ADCSRB |= (1 << ADTS0); //Sets timer 0 to trigger adc

  sei();  //enable interrupts
}




double myFFT(){
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
  return;
}

void ComputeMagnitude(){
  for (uint16_t i = 0; i <(SAMPLES >> 1) + 1; i++) {
    vReal[i] = sqrt(vReal[i]*vReal[i] + vImag[i]*vImag[i]);
  }
  return;
}

void HammingWindowing(){
  for (uint16_t i = 0; i < (SAMPLES >> 1); i++) {
    double ratio = (double(i) / (double(SAMPLES) - 1.0));
    double  weighingFactor = 0.54 - (0.46 * cos(2 * MYPI * ratio));
    vReal[i] *= weighingFactor;
    vReal[SAMPLES - (i + 1)] *= weighingFactor;
  }
  return;
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
      }
    }
  }

  //Interpolation
  if(IndexOfMaxY !=0 && vReal[IndexOfMaxY] > FFT_AMPLITUDE_TRESHOLD){
    double delta = ((- vReal[IndexOfMaxY-1] + vReal[IndexOfMaxY+1]) / (vReal[IndexOfMaxY-1] +  vReal[IndexOfMaxY] + vReal[IndexOfMaxY+1]));
    double interpolatedX = ((IndexOfMaxY + delta)  * SAMPLING_FREQUENCY) / (SAMPLES);
    return interpolatedX;
  }else{
    return 0.0;
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
  if(u < 10 && u >   0) u =  10;
  if(u < 0  && u > -10) u = -10;
  if(u >  60) u =  60;
  if(u < -60) u = -60;
  if(e_current < 0){
    if(-e_current < precision) u = 0;
  }else{
    if( e_current < precision) u = 0;
  }
  return int(u);
}


double frequencyCheck(double freq){
  if(freq>75.0 && freq <90.0){
    sevenSegment(E);
    precision = 0.3;
    return E2;   
  }
  if(freq>90.0 && freq <125.0){
    sevenSegment(A);
    precision = 0.3;
    return A2;
  }
  if(freq>125.0 && freq <165.0){
    sevenSegment(D);
    precision = 0.4;
    return D3;
  }
  if(freq>165.0 && freq <215.0){
    sevenSegment(G);
    precision = 0.5;
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

void flashLed(double diff){
  if(diff < 0){
    if(-diff < precision) {
      digitalWrite(LED_PIN_CENTER, 1);
      digitalWrite(LED_PIN_LEFT, 0);
      digitalWrite(LED_PIN_RIGHT, 0);
      return;
    }else{
      digitalWrite(LED_PIN_CENTER, 0);
      digitalWrite(LED_PIN_LEFT, 0);
      digitalWrite(LED_PIN_RIGHT, 1);
      return;
    }
  }else{
    if(diff < precision){
      digitalWrite(LED_PIN_CENTER, 1);
      digitalWrite(LED_PIN_LEFT, 0);
      digitalWrite(LED_PIN_RIGHT, 0);
      return;
    }else{
      digitalWrite(LED_PIN_CENTER, 0);
      digitalWrite(LED_PIN_LEFT, 1);
      digitalWrite(LED_PIN_RIGHT, 0);
      return;
    }
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
  return;
}


void pidReset(){
  e[0] = 0;
  e[1] = 0;
  e[2] = 0;
  u = 0.0;
  u_past = 0.0;
  return;
}

void displayClear(){
    PORTD = PORTD | B11000000;
    PORTB = PORTB | B11111111;
    digitalWrite(LED_PIN_CENTER, 0);
    digitalWrite(LED_PIN_LEFT, 0);
    digitalWrite(LED_PIN_RIGHT, 0);
    return;
}



 
void loop() {
  if (program_state == 2) {
    double peak = myFFT();
    if(peak != 0.0){
      double desired_peak = frequencyCheck(peak);
      double e_current = desired_peak - peak;
      int u_current = pid(e_current);
      flashLed(e_current);

      //Debugging
      char tmp[25];
      sprintf(tmp, "%u %d %u", uint16_t(peak*100),  u_current, uint16_t(desired_peak*100));      
      mySerial.println(tmp);
      //Debugging end

      while(time_counter < int(Tp*1000)*2); //waiting untill Tp is reached
      time_counter = 0;
      Serial.write(u_current); 
      adc_iterator = 0;
      display_counter = 0;
      program_state = 1;
    }else{
      displayClear();
      pidReset();
      Serial.write(0);
      program_state = 0;
    }
  }  
}

ISR(TIMER0_COMPA_vect) {};


ISR(ADC_vect) {
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
