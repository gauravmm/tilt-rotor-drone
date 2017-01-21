// Tilt-drone tilt controller
// Gaurav Manek
//
// This runs on an Arduino to intercept the roll, pitch, and mode switch
// from the RC receiver. It modifies the input, controls the tilt of the servo,
// and passes on the modified input to the flight controller. 

// The pitch used in MODE_FIXED, as the motor fraction.
#define FIXED_TILT 1600

#define RCI_THROTTLE 0
#define RCI_PITCH    1
#define RCI_MODE     2

#define MODE_DISABLE  0
#define MODE_FIXED    1
#define MODE_VARIABLE 2

#define WATCHDOG_TRIGGER 31
// The real-world tilt angle corresponding to maximum input value via the RC:
#define TILT_ANGLE   3.14159265/4 /* rad */

volatile unsigned long RC_START_TIME[3];
volatile unsigned long RC_VAL[3];

// This watchdog allows us to check if no pulses are detected on the RC input. 
volatile uint8_t updateWatchdog = 0;
uint8_t mode = MODE_DISABLE;

// Output pins are OC1A and OC1B
#define PIN_RC_THROTTLE 9
volatile uint16_t *VAL_RC_THROTTLE = OCR1A;
#define PIN_RC_PITCH    10
volatile uint16_t *VAL_RC_PITCH = OCR1B;
#define PIN_RC_TILT     15
volatile uint8_t CACHE_OCR2A = 0;
uint8_t *VAL_RC_TILT = &CACHE_OCR2A;

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(A0, INPUT_PULLUP);
  
  pinMode(LED_BUILTIN, OUTPUT);     // LED Pin
  pinMode(PIN_RC_THROTTLE, OUTPUT);
  pinMode(PIN_RC_PITCH, OUTPUT);
  pinMode(PIN_RC_TILT, OUTPUT);
  
  setupInterrupts();
}

void writeTiltPosition(uint16_t tilt) {
  // 0..4.080ms = 0..255
  // 1..2ms ~= 63..125
  // .: [1000..2000] -> [63..125]
  if (tilt <= 1000) {
    CACHE_OCR2A = 62;
  } else if (tilt >= 2000) {
    CACHE_OCR2A = 125;
  } else {
    // Map the input from domain [1000..2000] to range [63..125]; 
    CACHE_OCR2A = (tilt - 1000) / 16 + 63;
  }
}

void writeThrottlePosition(uint16_t thr_in, uint16_t tilt) {
  // First we express the tilt angle as a fraction between 
  float angle = 0.0;
  if (tilt <= 1000) {
    angle = -TILT_ANGLE;
  } else if (tilt >= 2000) {
    angle = TILT_ANGLE;
  } else {
    angle = (tilt-1500) * TILT_ANGLE / 500;
  }

  // We compensate for the loss of upward force (lift) due to the change in thrust vector 
  // direction by increasing the thrust to keep the lift constant. The excess thrust, thanks
  // to the tilting motors, goes into forward movement of the drone. 
  int16_t thrust_val = thr_in - 1500;
  thrust_val = thrust_val / cos(angle);
  
  if (thrust_val <= -500) {
    thrust_val = -500;
  } else if (thrust_val >= 500) {
    thrust_val = 500;
  }

  *VAL_RC_THROTTLE = 1500 + thrust_val; 
}

uint8_t itr = 0;
void loop() {
  // Interrupt-safe reading.
  cli();
  uint16_t inp_pitch = RC_VAL[RCI_PITCH];
  uint16_t inp_throttle = RC_VAL[RCI_THROTTLE];
  uint16_t inp_mode = RC_VAL[RCI_MODE];
  sei();

  // Update current mode.
  if(inp_mode < 1250) {
    mode = MODE_DISABLE;
  } else if(inp_mode < 1750) {
    mode = MODE_FIXED;
  } else {
    mode = MODE_VARIABLE;
  }


  // Check the watchdog for loss of signal, in an interrupt-safe manner:
  cli();
  if (updateWatchdog > WATCHDOG_TRIGGER) {
    updateWatchdog = WATCHDOG_TRIGGER;
    sei();

    *VAL_RC_THROTTLE = 0;
    // Reset the watchdog to prevent integer overflow.

  } else {
    sei();

    // Use the mode to set the output values.
    switch(mode){
      default:
      case MODE_DISABLE:
        writeTiltPosition(1500);
        *VAL_RC_PITCH = inp_pitch;
        *VAL_RC_THROTTLE = inp_throttle;

        // Blink based on mode
        digitalWrite(LED_BUILTIN, LOW);
        break;
      case MODE_FIXED:
        writeTiltPosition(FIXED_TILT);
        *VAL_RC_PITCH = inp_pitch;
        writeThrottlePosition(inp_throttle, FIXED_TILT);

        // Blink based on mode
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      case MODE_VARIABLE:
        
        writeTiltPosition(inp_pitch);
        *VAL_RC_PITCH = 1500;
        writeThrottlePosition(inp_throttle, inp_pitch);
        
        // Blink based on mode
        digitalWrite(LED_BUILTIN, (itr & 0b00001000)?HIGH:LOW);
        break;
    }
  }

  updateWatchdog++;
  itr++;
  delay(35);
}

void setupInterrupts() {
  // Prevent interrupts from being handled while in an inconsistent state.
  cli();
  // Enable external interrupt on Arduino pins 2, 3
  // Page 89-90 of manual
  EICRA  = EICRA  & ~_BV(ISC11) | _BV(ISC10) & ~_BV(ISC01) | _BV(ISC00);
  EIMSK  = EIMSK  |  _BV(INT1)  | _BV(INT0); 
  
  // Enable external interrupt on Arduino pin 8, as part of the pin bank C.
  PCICR  = PCICR  | _BV(PCIE1);
  PCMSK1 = PCMSK1 | _BV(PCINT8);

  // Set up Timer1 for RC output. 
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

  OCR1A = 0;
  OCR1B = 0;

  // Set up Timer2 for Servo output.
  // We can afford lower precision on the servo, so we use the 8-bit timer.

  // Set Phase Correct Mode 
  TCCR2A = TCCR2A & ~_BV(WGM21) | _BV(WGM20);
  TCCR2B = TCCR2B & ~_BV(WGM22);

  // Set /128 prescaler, so one timer period is 4.08ms
  TCCR2B = TCCR2B | _BV(CS22) & ~_BV(CS21) | _BV(CS20);

  // Set non-inverting output.
  TCCR2A = TCCR2A | _BV(COM2A1) & ~_BV(COM2A0);

  OCR2A = 0;

  sei();
}

// Update OCR2A from CACHE_OCR2A at the bottom, so we never miss a comparison.
ISR(TIMER2_OVF_vect) {
  OCR2A = CACHE_OCR2A;
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

  updateWatchdog = 0;
} 

// Handle the entire pin bank 0
ISR(PCINT1_vect) {
  intHandler(PINC & _BV(PINC0), RCI_MODE);
}

// Handle pin 2
ISR(INT0_vect) {
  intHandler(PIND & _BV(PIND2), RCI_PITCH);
}

// Handle pin 3
ISR(INT1_vect) {
  intHandler(PIND & _BV(PIND3), RCI_THROTTLE); 
}
