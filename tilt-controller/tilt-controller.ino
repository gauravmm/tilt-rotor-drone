// Tilt-drone tilt controller
// Gaurav Manek
//
// This runs on an Arduino to intercept the roll, pitch, and mode switch
// from the RC receiver. It modifies the input, controls the tilt of the servo,
// and passes on the modified input to the flight controller. 

#define RCI_THROTTLE 0
#define RCI_PITCH    1
#define RCI_MODE     2

unsigned long volatile RC_START_TIME[3];
unsigned long volatile RC_VAL[3];

// Output pins are OC1A and OC1B
#define PIN_RC_THROTTLE 9
#define PIN_RC_PITCH    10

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_RC_THROTTLE, OUTPUT);
  pinMode(PIN_RC_PITCH, OUTPUT);

  Serial.begin(115200);

  setupInterrupts();
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);

  Serial.println(RC_VAL[0]);
  Serial.println(RC_VAL[1]);
  Serial.println(RC_VAL[2]);
  Serial.println();
}

void setupInterrupts() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(8, INPUT);
  // Prevent interrupts from being handled while in an inconsistent state.
  cli();
  // Enable external interrupt on Arduino pins 2, 3
  // Page 89-90 of manual
  EICRA  = EICRA  & ~_BV(ISC11) | _BV(ISC10) & ~_BV(ISC01) | _BV(ISC00);
  EIMSK  = EIMSK  |  _BV(INT1)  | _BV(INT0); 
  
  // Enable external interrupt on Arduino pin 8, as part of the pin bank.
  PCICR  = PCICR  | _BV(PCIE0);
  PCMSK0 = PCMSK0 | _BV(PCINT0);

  // Set up Timer1 for servo output. 
  // We put it in Phase and Frequency Correct Mode, with TOP at ICR1 and OCR1x updated at BOTTOM
  // This way, we can set the same TOP value for both OC1[A:B], and change the pulse width using
  // OCR1[A:B] independently. 
  TCCR1A = TCCR1A & ~_BV(WGM11) & ~_BV(WGM10);
  TCCR1B = TCCR1B |  _BV(WGM13) & ~_BV(WGM12);

  // Since OCR1x is double-buffered and updated at BOTTOM, we can use
  // the ICF1 interrupt (set when TCNT1 reaches TOP) to zero OCR1x. This turns the PWM into a
  // single, high-precision pulse.  
  
  // We pick a /8 prescaler and TOP of 20,000. Assuming a 16MHz Arduino, this gives us a PWM 
  // frequency of 50Hz. The TOP value allows us to set time between 0..20ms to ~14 bits of
  // precision, which is equivalent to ~10 bits of precision in the 1..2ms range.
  //
  // We could get 3 more bits of precision in the 1..2ms range by using a prescaler of /1
  // and a TOP of 32,000 to give a PWM period of 4ms. However, we would have to write an ISR
  // to count the number of PWM periods and enable/disable output accordingly. That (1) would 
  // lead to clock drift over time, and (2) might interfere with the input timing ISR. (1) does
  // not matter much, the motors don't really care, but (2) is the major concern here.
  //
  // Also, those additional bits of precision do not matter as much. The input values are 
  // correct to 8-9 bits, and the trignometric functions are correct (depending on range) to
  // ~8 bits.
  ICR1 = 20000;
  TCCR1B = TCCR1B & ~_BV(CS12) | _BV(CS11) & ~_BV(CS10);

  // We enable regular output, so our OCR1[A:B] values in range 1000..2000 directly map 
  // to 1..2ms pulses.
  TCCR1A = TCCR1A | _BV(COM1A1) & ~_BV(COM1A0) | _BV(COM1B1) & ~_BV(COM1B0); 

  sei();
}

inline void intHandler(bool pinState, uint8_t rcIdx) {
  unsigned long time = micros();

  if(pinState) {
    RC_START_TIME[rcIdx] = time;
  } else {
    // Check if micros() has looped back. This happens every 50 minutes or so.
    if(time < RC_START_TIME[rcIdx]) {
      // If it has looped around, the delay has been (MAX_ULONG - START) + TIME 
      time = ~RC_START_TIME[rcIdx] + time;
    } else {
      time = time - RC_START_TIME[rcIdx];
    }

    // Basic sanity checking.
    if (time < 3000) {
      RC_VAL[rcIdx] = time;
    }
  }
} 

// Handle the entire pin bank 0
ISR(PCINT0_vect) {
  intHandler(PINB & _BV(PINB0), RCI_MODE);
}

// Handle pin 2
ISR(INT0_vect) {
  intHandler(PIND & _BV(PIND2), RCI_PITCH);
}

ISR(INT1_vect) {
  intHandler(PIND & _BV(PIND3), RCI_THROTTLE); 
}
