/* 
  Solar Bauble Unit Code 
  Framework: Watchdog Timer State Machine (Internal 1.1V Ref + Resistor Divider)
  Target MCU: ATTiny85 
  Fuses required: Extended: 0xFF | High: 0xD6 | Low: 0xE2 | Lock: 0xFF 
*/


#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

// Pin Assignments
#define pinLED0 PB0  // Pin 5 on ATtiny85
#define pinLED1 PB1  // Pin 6 on ATtiny85
#define pinLED2 PB3  // Pin 2 on ATtiny85
#define pinLED3 PB4  // Pin 3 on ATtiny85
#define sw      PB2  // Pin 7 on ATtiny85 (Analog Input A1)

// Configuration Parameters
// ADC Thresholds mapped via 100k/10k divider to 1.1V Internal Ref
const int ADC_PRIMING_LOW      = 50;   // ~0.5V: must drop below this to prime wake up
const int ADC_SLEEP_OFF        = 150;  // ~2.4V: forces absolute deep sleep
const int ADC_RED_ONLY_MIN     = 160;  // ~2.5V: minimum threshold for Red-only profile
const int ADC_RUNNING_HIGH     = 250;  // ~3.4V: triggers wake up & Red/Green mode

// Internal Vcc Thresholds (in Millivolts) for the Over-Voltage Dump Block
const long VCC_OVERVOLTAGE_HIGH = 5300; // 5.3V: Over-voltage protection trip point
const long VCC_OVERVOLTAGE_LOW  = 5200; // 5.2V: Over-voltage release point (0.1V hysteresis)

// State Machine Definitions
#define STATE_SLEEPING           0
#define STATE_RUNNING_RED_ONLY   1
#define STATE_RUNNING_FULL       2

// Global Variables
int currentState = STATE_SLEEPING; 
int lastLED = -1;
volatile bool primedToWake = false;
bool overVoltageActive = false;        // Track hysteresis state for over-voltage

// Structural index maps to eliminate while-loop traps entirely
const int redLEDs[]   = {1, 3, 5, 7}; // Odd matrix indexes
const int greenLEDs[] = {0, 2, 4, 6}; // Even matrix indexes

// Function Prototypes
void clearLED();
void setLED(int LED);
void wdt_sleep_delay(uint8_t prescaler);
void executeTwinklePattern(bool redOnly);
void executeOverVoltageBlinkSequence();
void setAllRed();
void setAllGreen();
long readVccInternal();

void setup() {
  pinMode(sw, INPUT);
  clearLED();
  
  // Set ADC Prescaler to 8 for stable 1MHz clock operations (ADPS1 and ADPS0)
  ADCSRA = (1 << ADPS1) | (1 << ADPS0);
  
  // Enable global interrupts so the Watchdog can wake up the CPU
  sei();
  
  // Give ADC voltage reference time to settle on boot
  delay(10); 
  
  // Check real capacitor voltage immediately at boot
  long initialVcc = readVccInternal();
  
  // Restore reference for external divider reading
  analogReference(INTERNAL1V1);
  delay(5);
  analogRead(A1); // Dummy read to force reference change
  int startupVoltage = analogRead(A1);
  
  if (initialVcc >= VCC_OVERVOLTAGE_HIGH) {
    overVoltageActive = true;
    primedToWake = true; // Auto-prime on over-voltage boot
  } else if (startupVoltage >= ADC_RUNNING_HIGH) {
    currentState = STATE_RUNNING_FULL;
    primedToWake = false; // Bypass priming if already fully charged on boot
  } else if (startupVoltage >= ADC_RED_ONLY_MIN) {
    currentState = STATE_RUNNING_RED_ONLY;
  } else {
    currentState = STATE_SLEEPING;
  }
}

void loop() {
  // Give ADC hardware time (5ms) to stabilize after waking up
  delay(5);
  
  // 1. HIGHEST PRIORITY INTERNAL VCC OVERVOLTAGE EVALUATION
  long vccValue = readVccInternal();
  
  if (!overVoltageActive) {
    if (vccValue >= VCC_OVERVOLTAGE_HIGH) {
      overVoltageActive = true;
    }
  } else {
    if (vccValue < VCC_OVERVOLTAGE_LOW) {
      overVoltageActive = false;
    }
  }

  // 2. OVER-VOLTAGE HIGH VISIBILITY ALTERNATING BLINK BLOCK
  if (overVoltageActive) {
    primedToWake = true; // Force wake priming true so state machine stays unblocked later
    executeOverVoltageBlinkSequence(); // Flash all Reds, 250ms pause, flash all Greens, 250ms pause
    return; // Intercept loop execution instantly to re-sample Vcc rail directly
  }

  // 3. RESTORE ADC FOR SW PIN METERING
  analogReference(INTERNAL1V1);
  delay(5); // Give internal hardware ref time to stabilize
  analogRead(A1); // Dummy read to flush multiplexer registers with new reference
  
  // 4. Sample the actual scaled potential divider voltage on Channel A1 (Physical Pin 7)
  int sensorValue = analogRead(A1);
  
  // 5. Track the low condition needed to prime a future wake up window
  if (sensorValue <= ADC_PRIMING_LOW) {
    primedToWake = true;
  }
  
  // 6. State Machine Transition Evaluation
  if (currentState == STATE_SLEEPING) {
    if (primedToWake && (sensorValue >= ADC_RUNNING_HIGH)) {
      currentState = STATE_RUNNING_FULL;
      primedToWake = false; // Reset priming flag only after verified full wake up
    }
  } else {
    if (sensorValue < ADC_SLEEP_OFF) {
      currentState = STATE_SLEEPING;
    } else if (sensorValue >= ADC_RUNNING_HIGH) {
      currentState = STATE_RUNNING_FULL;
    } else if (sensorValue >= ADC_RED_ONLY_MIN) {
      currentState = STATE_RUNNING_RED_ONLY;
    }
  }
  
  // 7. Execution Processing Blocks
  if (currentState == STATE_RUNNING_FULL || currentState == STATE_RUNNING_RED_ONLY) {
    bool redOnlyMode = (currentState == STATE_RUNNING_RED_ONLY);
    executeTwinklePattern(redOnlyMode);
  } else {
    // Under threshold / Locked out: Sleep for 4 seconds, then loop restarts to check SW
    wdt_sleep_delay(WDTO_4S);
  }
}

// Alternating block flash sequence for massive current dump and high visibility
void executeOverVoltageBlinkSequence() {
  // Phase A: Turn on all Red LEDs at high frequency
  for (int i = 0; i < 12; i++) { 
    setAllRed(); // Micro-strobe the 4 red channels sequentially
  }
  clearLED(); // High-Z block to prevent bleed
  wdt_sleep_delay(WDTO_250MS); // 250ms power-down sleep pause

  // Phase B: Turn on all Green LEDs at high frequency
  for (int i = 0; i < 12; i++) { 
    setAllGreen(); // Micro-strobe the 4 green channels sequentially
  }
  clearLED(); 
  wdt_sleep_delay(WDTO_250MS); // 250ms power-down sleep pause
}

// Micro-strobe helper to physically display all 4 Charlieplexed Red LEDs
void setAllRed() {
  for (int i = 0; i < 4; i++) {
    setLED(redLEDs[i]);
    delayMicroseconds(2500); 
  }
}

// Micro-strobe helper to physically display all 4 Charlieplexed Green LEDs
void setAllGreen() {
  for (int i = 0; i < 4; i++) {
    setLED(greenLEDs[i]);
    delayMicroseconds(1500); 
  }
}

// Modular Twinkle Execution Function
void executeTwinklePattern(bool redOnlyMode) {
  int selectedLED = -1;
  
  if (redOnlyMode) {
    int idx = random(0, 4);
    selectedLED = redLEDs[idx];
    if (selectedLED == lastLED) {
      idx = (idx + 1) % 4;
      selectedLED = redLEDs[idx];
    }
  } else {
    selectedLED = random(0, 8);
    if (selectedLED == lastLED) {
      selectedLED = (selectedLED + 1) % 8;
    }
  }
  
  lastLED = selectedLED;
  setLED(selectedLED);
  
  int randomOnChoice = random(0, 2);
  if (randomOnChoice == 0) {
    wdt_sleep_delay(WDTO_30MS);
  } else {
    wdt_sleep_delay(WDTO_60MS);
  }
  
  clearLED(); 
  
  int randomWaitChoice = random(0, 5);
  switch (randomWaitChoice) {
    case 0: wdt_sleep_delay(WDTO_120MS);  break;
    case 1: wdt_sleep_delay(WDTO_250MS); break;
    case 2: wdt_sleep_delay(WDTO_500MS); break;
    case 3: wdt_sleep_delay(WDTO_1S); break;
    case 4: wdt_sleep_delay(WDTO_2S);    break;
  }
}

// Secret Voltmeter Engine
long readVccInternal() {
  ADMUX = (0 << REFS1) | (0 << REFS0) | (1 << MUX3) | (1 << MUX2);
  delay(5); 
  ADCSRA |= (1 << ADSC); 
  while (bit_is_set(ADCSRA, ADSC)); 
  uint8_t low  = ADCL; 
  uint8_t high = ADCH; 
  long result = (high << 8) | low;
  result = 1126400L / result; 
  return result; 
}

// Watchdog Sleep Helper
void wdt_sleep_delay(uint8_t prescaler) {
  ADCSRA &= ~(1 << ADEN); 
  wdt_enable(prescaler);
  WDTCR |= (1 << WDIE);   
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
  sleep_disable();
  wdt_disable();
  ADCSRA |= (1 << ADEN);  
}

// Charlieplexing Matrix Drive Engine
void setLED(int LED) {
  switch (LED) {
    case 0:
      DDRB &= ~((1 << pinLED0) | (1 << pinLED1));
      DDRB |= (1 << pinLED2) | (1 << pinLED3);
      PORTB &= ~((1 << pinLED0) | (1 << pinLED1) | (1 << pinLED3));
      PORTB |= (1 << pinLED2);
      break;
    case 1:
      DDRB &= ~((1 << pinLED0) | (1 << pinLED1));
      DDRB |= (1 << pinLED2) | (1 << pinLED3);
      PORTB &= ~((1 << pinLED0) | (1 << pinLED1) | (1 << pinLED2));
      PORTB |= (1 << pinLED3);
      break;
    case 2:
      DDRB &= ~((1 << pinLED0) | (1 << pinLED3));
      DDRB |= (1 << pinLED1) | (1 << pinLED2);
      PORTB &= ~((1 << pinLED0) | (1 << pinLED2) | (1 << pinLED3));
      PORTB |= (1 << pinLED1);
      break;
    case 3:
      DDRB &= ~((1 << pinLED0) | (1 << pinLED3));
      DDRB |= (1 << pinLED1) | (1 << pinLED2);
      PORTB &= ~((1 << pinLED0) | (1 << pinLED1) | (1 << pinLED3));
      PORTB |= (1 << pinLED2);
      break;
    case 4:
      DDRB &= ~((1 << pinLED2) | (1 << pinLED3));
      DDRB |= (1 << pinLED0) | (1 << pinLED1);
      PORTB &= ~((1 << pinLED1) | (1 << pinLED2) | (1 << pinLED3));
      PORTB |= (1 << pinLED0);
      break;
    case 5:
      DDRB &= ~((1 << pinLED2) | (1 << pinLED3));
      DDRB |= (1 << pinLED0) | (1 << pinLED1);
      PORTB &= ~((1 << pinLED0) | (1 << pinLED2) | (1 << pinLED3));
      PORTB |= (1 << pinLED1);
      break;
    case 6:
      DDRB &= ~((1 << pinLED1) | (1 << pinLED2));
      DDRB |= (1 << pinLED0) | (1 << pinLED3);
      PORTB &= ~((1 << pinLED0) | (1 << pinLED1) | (1 << pinLED2));
      PORTB |= (1 << pinLED3);
      break;
    case 7:
      DDRB &= ~((1 << pinLED1) | (1 << pinLED2));
      DDRB |= (1 << pinLED0) | (1 << pinLED3);
      PORTB &= ~((1 << pinLED1) | (1 << pinLED2) | (1 << pinLED3));
      PORTB |= (1 << pinLED0);
      break;
  }
}

void clearLED() {
  DDRB &= ~((1 << pinLED0) | (1 << pinLED1) | (1 << pinLED2) | (1 << pinLED3));
  PORTB &= ~((1 << pinLED0) | (1 << pinLED1) | (1 << pinLED2) | (1 << pinLED3));
}

ISR(WDT_vect) {
  // Wake event hook
}


