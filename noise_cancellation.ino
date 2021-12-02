#include <DueTimer.h>

#define Fs 6000.0 // sampling frequency: Fs = 6000 Hz
#define _0 { 0, 0, 0, 0, 0, 0, 1 } // 7 Seg numbers
#define _1 { 1, 0, 0, 1, 1, 1, 1 }
#define _2 { 0, 0, 1, 0, 0, 1, 0 }
#define _3 { 0, 0, 0, 0, 1, 1, 0 }
#define _4 { 1, 0, 0, 1, 1, 0, 0 }
#define _5 { 0, 1, 0, 0, 1, 0, 0 }
#define _6 { 0, 1, 0, 0, 0, 0, 0 }
#define _7 { 0, 0, 0, 1, 1, 1, 1 }
#define _8 { 0, 0, 0, 0, 0, 0, 0 }
#define _9 { 0, 0, 0, 0, 1, 0, 0 }

volatile float t = 0.0, dt = 1.0/Fs, freq = 200.0, phase = 0.0, w = 0.0, r = 1.0;
volatile int sensorValue = 0, x1, x2, mic;

void pickDigit();
void pickNumber();
void clearLEDs();
int sineWave(float t, const float freq, float phase, float gain);
void sinewaveHandler();
void sw1Handler();
void sw2Handler();

/* 7 Seg */
int a = 37, b = 35, c = 33, d = 31, e = 29, f = 27, g = 25, p = 23; // Set anode pin
int d1 = 45, d2 = 43, d3 = 41, d4 = 39; // 7-Segment LED Display
int digitpin[5] = { -1, d4, d3, d2, d1 };

int num_bit[10][7] = { _0, _1, _2, _3, _4, _5, _6, _7, _8, _9 };

void pickDigit(int x) {
  digitalWrite(d1, LOW);
  digitalWrite(d2, LOW);
  digitalWrite(d3, LOW);
  digitalWrite(d4, LOW);
  digitalWrite(digitpin[x], HIGH);
}

void pickNumber(int n) {
  digitalWrite(a, num_bit[n][0]);
  digitalWrite(b, num_bit[n][1]);
  digitalWrite(c, num_bit[n][2]);
  digitalWrite(d, num_bit[n][3]);
  digitalWrite(e, num_bit[n][4]);
  digitalWrite(f, num_bit[n][5]);
  digitalWrite(g, num_bit[n][6]);
}

void clearLEDs() {
  digitalWrite(a, 1);
  digitalWrite(b, 1);
  digitalWrite(c, 1);
  digitalWrite(d, 1);
  digitalWrite(e, 1);
  digitalWrite(f, 1);
  digitalWrite(g, 1);
  digitalWrite(p, 1);
}


/* Signal Processing */
// Sine wave signal generator for Arduino due DAC: 0 <= output amplitude <= 4095.
int sineWave(float t, const float freq, float phase, float gain){
  // Sine wave generator of [-2048,2048]. y = A * sin(2*pi*f*t + phi)
  float sampled_sine = 2048.0*gain * sin(2.0*PI*freq*t + phase); 
  
  // Mapping of value in [-2048,2048] to [0,4095] for DAC output.
  int sampled_sine_offset = map( (int)sampled_sine, -2048, 2048, 0, 4095 ); 
  return sampled_sine_offset;
}

// Timer0 interrupt service routine (ISR) executed Fs times per second.
void sinewaveHandler() {
  t = t + dt;
  const float _freq = freq;
  const float _r = r;

  // Potentiometer reading from ADC.
  int sensorValue = analogRead(A2);
  
  // Conversion of int type potentiometer reading (0 ~ 4095) to float type phase variable (0 ~ 2π).
  phase = 2 * PI * ( ((float)sensorValue) / 4095.0 );

  // Generate sine wave x1 (fixed phase) and x2 (variable phase).
  x1 = sineWave(t, freq, 0, 1);
  x2 = sineWave(t, freq, phase, 1);

  // Output sine wave x1 and x2 to DAC0 and DAC1.
  analogWrite(DAC0, x1);
  analogWrite(DAC1, x2);

  // Read low pass filtered (LPF) values of x1 and x2 from ADC1 and ADC2.
  int y1 = analogRead(0);
  int y2 = analogRead(1);
}

// Switch 1 to increase the frequency by 10 Hz.
void sw1Handler(){
  static unsigned long last_interrupt_time1 = 0;
  unsigned long interrupt_time = millis();
  if(interrupt_time - last_interrupt_time1 > 200)
    freq = freq + 10;  
  last_interrupt_time1 = interrupt_time;
}

// Switch 2 to decrease the frequency by 10 Hz.
void sw2Handler(){
  static unsigned long last_interrupt_time2 = 0;
  unsigned long interrupt_time = millis();
  if(interrupt_time - last_interrupt_time2 > 200)
    freq = freq - 10;
  last_interrupt_time2 = interrupt_time;
}

void setup(){
  // Serial port baud rate setting.
  Serial.begin(9600);
  
  // Resolution setting for DAC and ADC to 12 bit.
  analogReadResolution(12);
  analogWriteResolution(12);
  
  // Timer0 for sine wave generator ISR setting. 
  // Timer0 frequency set to Fs Hz.
  Timer0.attachInterrupt(sinewaveHandler).setFrequency(Fs).start();

  // Switch pin mode for frequency manipulation reset.
  // Interrupt for each switch pin acts on the falling edge of clk.
  pinMode(52, INPUT); attachInterrupt(52, sw1Handler, FALLING);
  pinMode(53, INPUT); attachInterrupt(53, sw2Handler, FALLING);

  // 7-Segment pin mode reset.
  pinMode(d1, OUTPUT);
  pinMode(d2, OUTPUT);
  pinMode(d3, OUTPUT);
  pinMode(d4, OUTPUT);
  pinMode(a, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(c, OUTPUT);
  pinMode(d, OUTPUT);
  pinMode(e, OUTPUT);
  pinMode(f, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(p, OUTPUT);
}

void loop(){
  while(1){
    const float _freq = freq;
    // Diplay sine wave frequency on 7-seg display.
    if((int)_freq >= 100) {
      clearLEDs();
      pickDigit(2);
      pickNumber(((int)_freq / 100) % 10);
      delayMicroseconds(50);  
    }
    if((int)_freq >= 10){
      clearLEDs();
      pickDigit(3);
      pickNumber(((int)_freq / 10) % 10);
      delayMicroseconds(50);    
    }
    if((int)_freq >= 0){
      clearLEDs();
      pickDigit(4);
      pickNumber(((int)_freq) % 10);
      delayMicroseconds(50);  
    }
  }
}
