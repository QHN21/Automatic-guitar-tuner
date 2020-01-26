#define SAMPLES 128           
#define POWER 7
#define SAMPLING_FREQUENCY 2000

#define MYPI    3.141592653589793238462643383279502884197169

#define E6  81.7         // 82.40
#define A5 110.00        //110.00
#define D4 145.83        //146.83
#define G3 197.77        //196.00
#define B2 247.41        //246.94
#define E1 329.61        //329.63

#define A B01010000
#define B B00010011
#define D B00010110
#define E B00110001
#define G B00011001


#define LED_PIN_LEFT    5
#define LED_PIN_CENTER  4
#define LED_PIN_RIGHT   3
#define FFT_AMPLITUDE_TRESHOLD    100
 

//PID
double K = 4;
double Tp = 0.2;
double Ti = 1000000000;
double Td = 0;

double u = 0.0;
double u_past = 0.0;

double r[3] = {0.0, 0.0, 0.0};
double e[3] = {0.0, 0.0, 0.0};
//PID END

volatile double precision = 0.2;

double signal_real[SAMPLES];
double signal_imaginary[SAMPLES];

int program_state = 0;
int adc_iterator = 0;
int display_counter = 0;
volatile uint16_t time_counter = 0;

void setup() {
  Serial.begin(9600);
  
  //PID
  
  r[0] = K*(1 + Tp/(2*Ti)+Td/Tp);
  r[1] = K*(Tp/(2*Ti)-2*Td/Tp-1);
  r[2] = K*Td/Tp;

  //PID END
  
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
      tmp = signal_real[i];
      signal_real[i] = signal_real[j];
      signal_real[j] = tmp;
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
    uint16_t kn = 0;
    
    double cos_val = 1.0; //cos
    double sin_val = 0.0;  //sin

    for (j = 0; j < l1; j++) {
        cos_val = cos(-2*MYPI*kn/SAMPLES);
        sin_val = sin(-2*MYPI*kn/SAMPLES);

        kn += 1 << (POWER - l - 1);

       for (uint16_t i = j; i < SAMPLES; i += l2) {
          uint16_t i1 = i + l1;
          double t1 = cos_val * signal_real[i1] - sin_val * signal_imaginary[i1];
          double t2 = cos_val * signal_imaginary[i1] + sin_val * signal_real[i1];
          signal_real[i1] = signal_real[i] - t1;
          signal_imaginary[i1] = signal_imaginary[i] - t2;
          signal_real[i] += t1;
          signal_imaginary[i] += t2;
       }
    }
  }
  return;
}

void ComputeMagnitude(){
  for (uint16_t i = 0; i <(SAMPLES >> 1) + 1; i++) {
    signal_real[i] = sqrt(signal_real[i]*signal_real[i] + signal_imaginary[i]*signal_imaginary[i]);
  }
  return;
}

void HammingWindowing(){
  for (uint16_t i = 0; i < (SAMPLES >> 1); i++) {
    double ratio = (double(i) / (double(SAMPLES) - 1.0));
    double  weighingFactor = 0.54 - (0.46 * cos(2 * MYPI * ratio));
    signal_real[i] *= weighingFactor;
    signal_real[SAMPLES - (i + 1)] *= weighingFactor;
  }
  return;
}



double peakFind(){
  //Harmonic Product Spectrum Algorithm
  for(int i = 0; i< ((SAMPLES >> 1) + 1); i++)
    signal_imaginary[i] = signal_real[i];
  for(int i = 0; i*2< ((SAMPLES >> 1) + 1); i++)
    signal_imaginary[i] = signal_imaginary[i] * signal_real[i*2];
  for(int i = 0; i*3< ((SAMPLES >> 1) + 1); i++)
    signal_imaginary[i] = signal_imaginary[i] * signal_real[i*3];

   //Peak find  
  double maxY = 0;
  uint16_t IndexOfMaxY = 0;
  for (uint16_t i = 4; i < ((SAMPLES >> 1) + 1); i++) {
    if ((signal_imaginary[i-1] < signal_imaginary[i]) && (signal_imaginary[i] > signal_imaginary[i+1])) {
      if (signal_imaginary[i] > maxY) {
        maxY = signal_imaginary[i];
        IndexOfMaxY = i;
      }
    }
  }

  //Barycentric Interpolation
  if(IndexOfMaxY !=0 && signal_real[IndexOfMaxY] > FFT_AMPLITUDE_TRESHOLD){
    double delta = ((- signal_real[IndexOfMaxY-1] + signal_real[IndexOfMaxY+1]) / (signal_real[IndexOfMaxY-1] +  signal_real[IndexOfMaxY] + signal_real[IndexOfMaxY+1]));
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
  double u_tmp = u;
  
  //Constraints
  if(u_tmp < 10 && u_tmp >   0) u_tmp =  10;
  if(u_tmp < 0  && u_tmp > -10) u_tmp = -10;
  if(u_tmp >  60) u_tmp =  60;
  if(u_tmp < -60) u_tmp = -60;
  if(e_current < 0){
    if(-e_current < precision) u_tmp = 0;
  }else{
    if( e_current < precision) u_tmp = 0;
  }
  return int(u_tmp);
}


double frequencyCheck(double freq){
  if(freq>50 && freq <90.0){
    sevenSegment(E);
    precision = 0.5;
    return E6;   
  }
  if(freq>90.0 && freq <125.0){
    sevenSegment(A);
    precision = 0.5;
    return A5;
  }
  if(freq>125.0 && freq <165.0){
    sevenSegment(D);
    precision = 0.5;
    return D4;
  }
  if(freq>165.0 && freq <215.0){
    sevenSegment(G);
    precision = 0.6;
    return G3;
  }
  if(freq>215.0 && freq <275.0){
    sevenSegment(B);
    precision = 0.7;
    return B2;
  }
  if(freq>275.0){
    sevenSegment(E);
    precision = 0.8;
    return E1;   
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
      while(time_counter <= int(Tp*1000)*2); //waiting untill Tp is reached
      time_counter = 0;
      Serial.write(u_current); 
      adc_iterator = 0;
      display_counter = 0;
      program_state = 1;
    }else{
      //pulse_k = 0;
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
     signal_real[adc_iterator] = ADC;
     signal_imaginary[adc_iterator++] = 0;
    if (adc_iterator == SAMPLES) {
      adc_iterator = 0;
      program_state = 2;
    }  
  }
   time_counter++;
}
