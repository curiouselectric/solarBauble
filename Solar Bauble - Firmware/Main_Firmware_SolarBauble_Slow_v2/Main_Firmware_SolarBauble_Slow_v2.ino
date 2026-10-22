/* 
  Solar Bauble Unit Code 
  Framework: Watchdog Timer State Machine (Internal 1.1V Ref + Resistor Divider)
  Target MCU: ATTiny85 
  Fuses required: Extended: 0xFF | High: 0xD6 | Low: 0xE2 | Lock: 0xFF 
*/

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

// --- Pin Assignments ---
#define pinLED0 PB0  // Pin 5 on ATTiny85 chip
#define pinLED1 PB1  // Pin 6 on ATTiny85 chip
#define pinLED2 PB3  // Pin 2 on ATTiny85 chip
#define pinLED3 PB4  // Pin 3 on ATTiny85 chip
#define sw      PB2  // Pin 7 on ATTiny85 chip (Maps natively to Analog Input A1)

// --- Configuration Parameters (ADC Thresholds mapped via 100k/10k divider to 1.1V Internal Ref) ---
const int ADC_PRIMING_LOW  = 50;   // ~0.5V real voltage: must drop below this to prime wake-up
const int ADC_SLEEP_OFF    = 180;  // ~2.4V real voltage: dropping below this forces absolute deep sleep
const int ADC_RED_ONLY_MIN = 200;  // ~2.5V real voltage: minimum threshold to run the Red-only profile
const int ADC_RUNNING_HIGH = 250;  // ~3.4V real voltage: must exceed this to trigger wake-up & Red/Green mode

// --- State Machine Definitions ---
#define STATE_SLEEPING          0
#define STATE_RUNNING_RED_ONLY  1
#define STATE_RUNNING_FULL      2

// --- Global Variables ---
int currentState = STATE_SLEEPING; // Will be set dynamically during setup()
int lastLED = -1;       
volatile bool primedToWake = false; 

// Lists of structural index maps to eliminate while loop traps entirely
const int redLEDs[]   = {1, 3, 5, 7}; // Odd indexes
const int greenLEDs[] = {0, 2, 4, 6}; // Even indexes

void setup() {
  pinMode(sw, INPUT);
  clearLED();
  
  // Lock the ADC reference to the stable internal 1.1V bandgap
  analogReference(INTERNAL1V1);

  // Set ADC Prescaler to 8 for stable 1MHz clock operations
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0); 
  ADCSRA &= ~(1 << ADPS2);

  // Enable global interrupts so the Watchdog can wake up the CPU
  sei();

  // CRITICAL STARTUP FIX: Check the real capacitor voltage immediately at boot.
  // If the voltage is already high, bypass the priming lockout and start running.
  delay(10); // Give ADC voltage reference time to settle on boot
  int startupVoltage = analogRead(A1);
  
  if (startupVoltage > ADC_RUNNING_HIGH) {
    currentState = STATE_RUNNING_FULL;
  } else if (startupVoltage > ADC_RED_ONLY_MIN) {
    currentState = STATE_RUNNING_RED_ONLY;
  } else {
    currentState = STATE_SLEEPING;
  }
}

void loop() {
  // Give ADC hardware time (~5ms) to stabilize after waking up
  delay(5); 

  // 1. Sample the scaled potential divider voltage on Channel A1 (Physical Pin 7 / PB2).
  int sensorValue = analogRead(A1);

  // 2. Track the low condition needed to prime a future wake-up window.
  if (sensorValue < ADC_PRIMING_LOW) {
    primedToWake = true;
  }

  // 3. State Machine Transition Evaluation
  if (currentState == STATE_SLEEPING) {
    // Only wake up if it previously dropped below priming low AND now crosses above high threshold (3.4V)
    if (primedToWake && (sensorValue > ADC_RUNNING_HIGH)) {
      currentState = STATE_RUNNING_FULL;
      primedToWake = false; // Reset priming flag ONLY after full wake-up success is confirmed
    }
  } 
  else {
    // If the unit is currently active, re-evaluate its state tiers dynamically
    if (sensorValue < ADC_SLEEP_OFF) {
      currentState = STATE_SLEEPING; // Drops into deep sleep below 2.4V
    } 
    else if (sensorValue > ADC_RUNNING_HIGH) {
      currentState = STATE_RUNNING_FULL; // Red & Green active above 3.4V
    } 
    else if (sensorValue > ADC_RED_ONLY_MIN) { 
      currentState = STATE_RUNNING_RED_ONLY; // Red only active between 2.5V and 3.4V
    }
  }

  // 4. Execution Processing Blocks based on current resolved state
  if (currentState == STATE_RUNNING_FULL || currentState == STATE_RUNNING_RED_ONLY) {
    bool redOnlyMode = (currentState == STATE_RUNNING_RED_ONLY);
    int selectedLED = -1;

    // LOCK-PROOF RED ONLY SELECTION
    if (redOnlyMode) {
      int idx = random(0, 4);
      selectedLED = redLEDs[idx];
      
      if (selectedLED == lastLED) {
        idx = (idx + 1) % 4;
        selectedLED = redLEDs[idx];
      }
    } 
    // LOCK-PROOF FULL SELECTION (Red & Green)
    else {
      selectedLED = random(0, 8);
      if (selectedLED == lastLED) {
        selectedLED = (selectedLED + 1) % 8;
      }
    }

    lastLED = selectedLED;
    setLED(selectedLED);

    // Quick burst on-time
    int randomOnChoice = random(0, 2);
    if (randomOnChoice == 0) wdt_sleep_delay(WDTO_30MS);
    else                     wdt_sleep_delay(WDTO_60MS);

    clearLED();

    // Variable off-time dark pause
    int randomWaitChoice = random(0, 5);
    switch (randomWaitChoice) {
      case 0: wdt_sleep_delay(WDTO_60MS);  break;
      case 1: wdt_sleep_delay(WDTO_120MS); break;
      case 2: wdt_sleep_delay(WDTO_250MS); break;
      case 3: wdt_sleep_delay(WDTO_500MS); break;
      case 4: wdt_sleep_delay(WDTO_1S);    break;
    }
  } 
  else {
    // If state resolved to STATE_SLEEPING, execute an 8-second low-power block
    system_sleep_8s();
  }
}

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

void system_sleep_8s() {
  ADCSRA &= ~(1 << ADEN);  
  wdt_enable(WDTO_8S);   
  WDTCR |= (1 << WDIE);  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();                       
  sleep_cpu();                          
  sleep_disable();        
  wdt_disable();          
  ADCSRA |= (1 << ADEN);  
}

ISR(WDT_vect) {
  // Empty hook to cleanly handle waking up the core
}

void setLED(int LED) {
  switch (LED) {
    case 0:
      DDRB &= ~(1 << pinLED0); DDRB &= ~(1 << pinLED1); DDRB |= (1 << pinLED2) | (1 << pinLED3);
      PORTB &= ~((1 << pinLED0) | (1 << pinLED1) | (1 << pinLED3)); PORTB |= (1 << pinLED2);
      break;
    case 1:
      DDRB &= ~(1 << pinLED0); DDRB &= ~(1 << pinLED1); DDRB |= (1 << pinLED2) | (1 << pinLED3);
      PORTB &= ~((1 << pinLED0) | (1 << pinLED1) | (1 << pinLED2)); PORTB |= (1 << pinLED3);
      break;
    case 2:
      DDRB &= ~(1 << pinLED0); DDRB &= ~(1 << pinLED3); DDRB |= (1 << pinLED1) | (1 << pinLED2);
      PORTB &= ~((1 << pinLED0) | (1 << pinLED2) | (1 << pinLED3)); PORTB |= (1 << pinLED1);
      break;
    case 3:
      DDRB &= ~(1 << pinLED0); DDRB &= ~(1 << pinLED3); DDRB |= (1 << pinLED1) | (1 << pinLED2);
      PORTB &= ~((1 << pinLED0) | (1 << pinLED1) | (1 << pinLED3)); PORTB |= (1 << pinLED2);
      break;
    case 4:
      DDRB &= ~(1 << pinLED2); DDRB &= ~(1 << pinLED3); DDRB |= (1 << pinLED0) | (1 << pinLED1);
      PORTB &= ~((1 << pinLED1) | (1 << pinLED2) | (1 << pinLED3)); PORTB |= (1 << pinLED0);
      break;
    case 5:
      DDRB &= ~(1 << pinLED2); DDRB &= ~(1 << pinLED3); DDRB |= (1 << pinLED0) | (1 << pinLED1);
      PORTB &= ~((1 << pinLED0) | (1 << pinLED2) | (1 << pinLED3)); PORTB |= (1 << pinLED1);
      break;
    case 6:
      DDRB &= ~(1 << pinLED1); DDRB &= ~(1 << pinLED2); DDRB |= (1 << pinLED0) | (1 << pinLED3);
      PORTB &= ~((1 << pinLED0) | (1 << pinLED1) | (1 << pinLED2)); PORTB |= (1 << pinLED3);
      break;
    case 7:
      DDRB &= ~(1 << pinLED1); DDRB &= ~(1 << pinLED2); DDRB |= (1 << pinLED0) | (1 << pinLED3);
      PORTB &= ~((1 << pinLED1) | (1 << pinLED2) | (1 << pinLED3)); PORTB |= (1 << pinLED0);
      break;
  }
}

void clearLED() {
  DDRB &= ~((1 << pinLED0) | (1 << pinLED1) | (1 << pinLED2) | (1 << pinLED3));
  PORTB &= ~((1 << pinLED0) | (1 << pinLED1) | (1 << pinLED2) | (1 << pinLED3));
}
