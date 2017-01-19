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

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

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
