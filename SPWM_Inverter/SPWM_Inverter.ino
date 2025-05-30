/*
  Original Source: https://github.com/Irev-Dev/Arduino-Atmel-sPWM/blob/master/sPWM_generate_lookup_table/sPWM_generate_lookup_table.ino
  Modified By Sourav Goswami (https://github.com/Souravgoswami/)
*/
#pragma GCC optimize "Os"
#include <stdbool.h>

#define DEBUG false

/*
  Sine Wave tweaks
*/
// Sub divisions of sisusoidal wave, set it to 10 for 10 * 50 (freq) = 500Hz, and 40 for 2kHz switching speed
// I'm using 13 SWG pure copper wire, having a skin depth of 1190um at 3kHz, which is on the edge.
// Having more frequency or bunching up multiple wires together during transformer winding will increase the overall waveform and the efficiency
// NOTE THAT HAVING LOWER SIN_DIVISIONS (e.g. 10) WILL OSCILLATE THE OUTPUT. PLEASE ADJUST PID CONTROL PARAMETERS TO AVOID THAT.
#define SIN_DIVISIONS 40
#define POWER_CONFIRMATION_DELAY 10
// Sinusoidal frequency
#define FREQUENCY 50

// Adjust based on your transformer and output capacitor
#define MAX_PWM_ADJ 1.25
#define MIN_PWM_ADJ 0.01
#define TARGET_VOLTAGE 210.0
#define TEST_DC_OUTPUT_AFTER_STARTUP_DURATION 1000

/*
  AC/DC Voltage Detection
*/
// Collect samples for 40ms (for 50Hz sine wave fully rectified it's 40 / 10ms = 4 cycles, good enough, not too slow)
#define ADC_RES_UPDATE_INTERVAL 10000
#define AC_VOLTAGE_MEASUREMENT_CORRECTION_FACTOR 1
#define DC_VOLTAGE_MEASUREMENT_CORRECTION_FACTOR 1

#define SAMPLING_WINDOW_MS 60
#define RESISTOR_DIVIDER_1 1000000
#define RESISTOR_DIVIDER_2 10000

/*
  Relay AC Side Switching Going to AC
*/
#define DELAY_MS_BEFORE_RELAY_TOGGLES_FROM_DC_TO_DC 2000
#define SHUTDOWN_COOLDOWN_PERIOD_AFTER_AC_RECOVERY 1000
#define RELAY_PIN 3

#define BUZZER_PIN 4
// Duration of each beep in ms
#define BUZZ_DURATION 120
// Pause between two beeps
#define BUZZ_PAUSE 80
// Number of beeps in each sequence
#define BUZZ_COUNT 2
#define BUZZ_REPEAT_INTERVAL 5000

/*
  Software-based undervoltage detection and controlled shutdown
  Monitors DC input voltage via voltage divider on analog pin A2
  If voltage drops below 11V, Arduino will shut down the DC UPS
*/

// First warning threshold
#define WIRE_DROP 0.5
#define UV_WARN_LEVEL_1 11.5 - WIRE_DROP
// Second warning threshold
#define UV_WARN_LEVEL_2 11.2 - WIRE_DROP
// Shutdown threshold
#define UV_SHUTDOWN_LEVEL 10.7 - WIRE_DROP
// Upper resistor of voltage divider. Precision required
#define V_SENSE_R1_VDIV 100000
// Lower resistor of voltage divider. Precision required
#define V_SENSE_R2_VDIV 22000
#define ADC_INPUT_DC_VOLTAGE ((1 << REFS0) | 1)
#define BUZZER_ENABLED true

/*
  LED Configuration
*/

#define LED_INDICATION_PIN 8
#define LED_WARNING_PIN 7
#define FLASH_ON_UV_WARN true

/*
  AC Detection
*/
#define AC_DETECTION_OPTOCOUPLER_OUTPUT_PIN 2
#define ADC_INPUT_AC_VOLTAGE ((1 << REFS0) | 0)

/*
  Current Sense with ACS 712
*/
#define ADC_INPUT_I_SENSE ((1 << REFS0) | 2)
#define ACS_712_ZERO_CURRENT_VOLTAGE 2.5
#define ACS_712_CALIBRATION_FACTOR 1.01
#define ACS_712_VOLTAGE_PER_AMPS 0.185

/*
  Over-current Protection
*/

#define OCP_CURRENT 3
#define OCP_PERMISSIBLE_MS 250

// Do not modify
#define OUTPUT_A 9
#define OUTPUT_B 10

#include <avr/io.h>
#include <avr/interrupt.h>
#include "libraries/millis64/millis64.h"
#include "libraries/buzzer/buzzer.h"
#include "libraries/led/led.h"

// AC voltage level (transformer's isolated output)
struct ACVoltageData {
  float rmsVoltage = 0;
  float voltage = 0;
  float avgACVoltage = 0;
  float voltageDividerGain = ((float)RESISTOR_DIVIDER_1 + (float)RESISTOR_DIVIDER_2) / (float)RESISTOR_DIVIDER_2;
};

// AC voltage level (transformer's isolated output)
struct OutputCurrentSenseData {
  float acs712ZeroCurrentVoltage = 5.0 / 2.0;
  float rmsCurrent = 0;
};

// DC voltage level (from battery directly)
struct DCVoltageData {
  static constexpr float hysteresis = 0.1;
  float currentVoltage = 0;
  float voltageDividerGain = ((float)V_SENSE_R1_VDIV + (float)V_SENSE_R2_VDIV) / (float)V_SENSE_R2_VDIV;
};

// Buzzer implementation
struct BuzzerData {
  uint64_t nextToggleTime = 0;
  uint64_t lastSequenceTime = 0;
  uint16_t beepIndex = 0;
  bool buzzerOn = false;
  bool inBeepSequence = false;
};

// SPWM data
struct SPWMControlData {
  volatile uint32_t waveformScale = (uint32_t)roundf(MIN_PWM_ADJ * 128.0);
  volatile uint16_t pwmStepIndex = 0;
  volatile uint16_t lookUp[SIN_DIVISIONS / 2] = { 0 };
  const uint16_t half_sinDivisions = SIN_DIVISIONS / 2;
  static constexpr uint32_t pwmDutyClampMinPercent = 5;
  static constexpr uint32_t pwmDutyClampMaxPercent = 95;
  volatile uint16_t pwmLowClampThreshold = 0;
  volatile uint16_t pwmHighClampThreshold = 0;
  volatile uint8_t TCCR1AValue = (1 << COM1A1) | (1 << WGM11);
  static constexpr uint8_t output_a_on_mask = (1 << COM1A1);
  static constexpr uint8_t output_b_on_mask = (1 << COM1B1);
  volatile bool pwmToggleDelay = false;
};

// UPS control data
struct UPSData {
  uint64_t lastChangeTime = 0;
  uint64_t lastSwitchedToDCOffset = 0;
  uint64_t testDCOutputOffset = 0;
  uint64_t ocpTriggeredAt = false;
  uint64_t acSwitchedLastAt = 0;
  uint8_t currentWarnLevel = 0;
  bool powerOut = false;
  bool shutdown = false;
  bool ocpTriggered = false;
  bool ocpStart = false;
  volatile bool switchToAC = false;
};

// ADC information
struct ADCData {
  float adcRes = 0;
  uint64_t adcResLastUpdatedAtWithOffset = 0;
};

// PID Controller Data
struct PIDControlData {
  float filteredError = 0;
  float integrator = 0;
  float previousError = 0;

  // --- PID gains ---
  // Proportional gain
  static constexpr float kp = 0.025;
  // Integral gain
  static constexpr float ki = 0.0005;
  // Derivative gain
  static constexpr float kd = 0.0001;
};

ACVoltageData acVoltage;
OutputCurrentSenseData outputCurrentSense;
DCVoltageData dcVoltage;
BuzzerData buzzer;
SPWMControlData spwmControl;
PIDControlData pidControl;
ADCData adcData;
UPSData upsData;

// Get Vcc in V
float readVcc() {
  // Save previous settings
  uint8_t prevADMUX = ADMUX;

  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delayMicroseconds(500);

  // Discard result
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC)) {};

  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC)) {};

  uint32_t result = ADCL;
  result |= ADCH << 8;

  // Restore previous settings
  ADMUX = prevADMUX;

  // 1.1 * 1023 / result
  return 1125.3 / result;
}

void setup() {
  delay(100);

  // Configure and beep 12V Buzzer
  Buzzer::begin(BUZZER_PIN, LED_WARNING_PIN);
  LED::begin(LED_INDICATION_PIN);

  Buzzer::startBeep();
  LED::startFlash();

  delay(100);

  #if DEBUG
    Serial.begin(115200);
  #endif

  Buzzer::stopBeep();
  LED::stopFlash();

  // Period of PWM in clock cycles
  uint16_t period = F_CPU / FREQUENCY / SIN_DIVISIONS;

  spwmControl.pwmLowClampThreshold = (period * spwmControl.pwmDutyClampMinPercent) / 100;
  spwmControl.pwmHighClampThreshold = (period * spwmControl.pwmDutyClampMaxPercent) / 100;

  // Unused pins - set as INPUT_PULLUP to avoid floating
  const uint8_t unusedPins[] = { A3, A4, A5, A6, A7, 12, 11, 6, 5 };

  for (uint8_t i = 0; i < sizeof(unusedPins) / sizeof(unusedPins[0]); i++) {
    pinMode(unusedPins[i], INPUT_PULLUP);
  }

  // Generate the look up table, offset first and last cycles
  for (uint16_t i = 1; i < spwmControl.half_sinDivisions; i++) {
    float theta = M_PI * (i - 1) / (spwmControl.half_sinDivisions - 2);  // skip 0
    uint16_t value = (sin(theta) + 1) * 0.5 * period;
    spwmControl.lookUp[i] = value;
  }

  // Register initilisation, see datasheet for more detail.
  // ADC Enable, Prescaler=128
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  delay(10);

  // Calibrate zero current voltage (no load) for ACS712
  uint64_t sampleEndTime = millis64() + SAMPLING_WINDOW_MS;
  uint32_t numSamples = 0;
  uint32_t sum = 0;
  float vcc = readVcc();

  while (millis64() < sampleEndTime) {
    // Select ADC0 (A2), AVCC as reference
    ADMUX = ADC_INPUT_I_SENSE;
    ADCSRA |= (1 << ADSC);
    // Discard result
    while (bit_is_set(ADCSRA, ADSC)) {};
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {};
    sum += ADC;

    numSamples++;
  }

  outputCurrentSense.acs712ZeroCurrentVoltage = (sum / (float)numSamples) * (vcc / 1023.0);
  // Finish Zero current calibration

  // Other register initializations
  ICR1 = period;
  TCNT1 = 0;
  TCCR1A = spwmControl.TCCR1AValue;
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);

  delay(10);

  // Enable Timer1 overflow interrupt (triggers ISR(TIMER1_OVF_vect) when Timer1 overflows)
  TIMSK1 = (1 << TOIE1);

  // Enable Timer2 compare match A interrupt (triggers ISR(TIMER2_COMPA_vect) when OCR2A matches) for buzzer and LED indication light
  TIMSK2 = (1 << OCIE2A);

  // Set PB1 and PB2 as outputs.
  DDRB |= (1 << PB1) | (1 << PB2);

  // AC signal present or not - output from optocoupler
  pinMode(AC_DETECTION_OPTOCOUPLER_OUTPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(AC_DETECTION_OPTOCOUPLER_OUTPUT_PIN), powerMonitorISR, CHANGE);

  // MOSFET Gate Pin for AC/Battery 220V output DPDT Relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // Make pin 9, 10 output to ensure they don't float when inactive
  pinMode(OUTPUT_A, OUTPUT);
  pinMode(OUTPUT_B, OUTPUT);

  digitalWrite(OUTPUT_A, LOW);
  digitalWrite(OUTPUT_B, LOW);

  delay(TEST_DC_OUTPUT_AFTER_STARTUP_DURATION);

  for (uint8_t i = 0; i < 3; ++i) {
    Buzzer::startBeep();
    LED::startFlash();
    delay(45);
    Buzzer::stopBeep();
    LED::stopFlash();
    delay(45);
  }

  delay(200);
}

void startInverter() {
  // Turn off the relay and run the SPWM ISR code
  digitalWrite(3, LOW);
  uint64_t now = millis64();

  if ((dcVoltage.currentVoltage <= UV_SHUTDOWN_LEVEL) && (now >= upsData.acSwitchedLastAt + SHUTDOWN_COOLDOWN_PERIOD_AFTER_AC_RECOVERY)) {
    upsData.shutdown = true;
  }

  if (!upsData.shutdown && !upsData.ocpTriggered && outputCurrentSense.rmsCurrent >= OCP_CURRENT) {
    upsData.ocpTriggeredAt = now;
    upsData.ocpTriggered = true;
  }

  if (now >= upsData.ocpTriggeredAt + OCP_PERMISSIBLE_MS && !upsData.ocpStart) {
    if (outputCurrentSense.rmsCurrent >= OCP_CURRENT) {
      upsData.ocpStart = true;
    } else {
      upsData.ocpStart = false;
    }
  }

  if (dcVoltage.currentVoltage <= UV_SHUTDOWN_LEVEL) {
    upsData.currentWarnLevel = 3;
  } else if (dcVoltage.currentVoltage <= UV_WARN_LEVEL_2 - dcVoltage.hysteresis) {
    upsData.currentWarnLevel = 2;
  } else if (dcVoltage.currentVoltage <= UV_WARN_LEVEL_1 - dcVoltage.hysteresis) {
    upsData.currentWarnLevel = 1;
  } else if (dcVoltage.currentVoltage >= UV_WARN_LEVEL_1 + dcVoltage.hysteresis) {
    upsData.currentWarnLevel = 0;
  }

  // Buzzer alarm on drained battery
  if (!upsData.shutdown && !upsData.ocpStart) {
    Buzzer::resume();
  }

  if (upsData.shutdown && !Buzzer::isShutdown()) {
    Buzzer::triggerShutdownAlarm(1000);
  } else if (upsData.currentWarnLevel == 2) {
    Buzzer::beep(5, 40, 60, 2000);
  } else if (upsData.currentWarnLevel == 1) {
    Buzzer::beep(3, 40, 60, 9720);
  } else if (upsData.currentWarnLevel == 0) {
    Buzzer::beep(2, 160, 240, 30000);
  }

  // LED alarm on drained battery
  if (FLASH_ON_UV_WARN) {
    LED::resume();

    if (upsData.shutdown) {
      LED::flash(3, 100, 100, 300);
    }
  }

  if (!(upsData.shutdown || upsData.ocpStart)) {
    // --- Compute error ---
    float error = TARGET_VOLTAGE - acVoltage.avgACVoltage;

    // --- Filter error (simple exponential smoothing) ---
    pidControl.filteredError = 0.9 * pidControl.filteredError + 0.1 * error;

    // --- Derivative term ---
    float derivative = (pidControl.filteredError - pidControl.previousError) / 0.06;  // assuming 60ms loop
    pidControl.previousError = pidControl.filteredError;

    // --- Predict output (before clamping) ---
    float pidOutput = pidControl.kp * pidControl.filteredError + pidControl.integrator + pidControl.kd * derivative;

    // --- Anti-windup: only integrate if output will remain within bounds ---
    float tentativeIntegrator = pidControl.integrator + pidControl.filteredError * pidControl.ki;
    float tentativeOutput = pidControl.kp * pidControl.filteredError + tentativeIntegrator + pidControl.kd * derivative;

    if (tentativeOutput <= MAX_PWM_ADJ && tentativeOutput >= MIN_PWM_ADJ) {
      pidControl.integrator = tentativeIntegrator;
      pidOutput = tentativeOutput;
    }

    // --- Clamp final output ---
    if (pidOutput > MAX_PWM_ADJ) pidOutput = MAX_PWM_ADJ;
    if (pidOutput < MIN_PWM_ADJ) pidOutput = MIN_PWM_ADJ;

    spwmControl.waveformScale = (uint32_t)roundf(pidOutput * 128.0);
  } else {
    spwmControl.waveformScale = (uint32_t)roundf(MIN_PWM_ADJ * 128.0);
  }
}

void loop() {
  if (upsData.ocpStart) {
    Buzzer::startBeep();
    LED::startFlash();

    // Turn off relay (as inverter is turned off, this will disconnect the output)
    digitalWrite(3, LOW);

    return;
  }

  uint64_t now = millis64();

  if (now > adcData.adcResLastUpdatedAtWithOffset) {
    adcData.adcResLastUpdatedAtWithOffset = now + ADC_RES_UPDATE_INTERVAL;
    adcData.adcRes = readVcc() / 1023.0;
  }

  float sumSquaredVoltage = 0;
  float sumSquaredCurrent = 0;
  uint32_t numSamples = 0;

  uint64_t sampleEndTime = millis64() + SAMPLING_WINDOW_MS;

  while (millis64() < sampleEndTime) {
    // Select ADC0 (A0), AVCC as reference
    ADMUX = ADC_INPUT_AC_VOLTAGE;
    delayMicroseconds(5);

    // Discard result
    ADCSRA |= (1 << ADSC);
    while (bit_is_set(ADCSRA, ADSC)) {};

    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {};
    float voltage = ADC * adcData.adcRes;
    sumSquaredVoltage += voltage * voltage;

    delayMicroseconds(10);

    // Select ADC0 (A2), AVCC as reference
    ADMUX = ADC_INPUT_I_SENSE;
    delayMicroseconds(5);

    // Discard result
    ADCSRA |= (1 << ADSC);
    while (bit_is_set(ADCSRA, ADSC)) {};

    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {};
    voltage = ADC * adcData.adcRes;

    float centeredVoltage = voltage - outputCurrentSense.acs712ZeroCurrentVoltage;
    // float instCurrent = centeredVoltage < 0 ? 0 : centeredVoltage / ACS_712_VOLTAGE_PER_AMPS;
    float instCurrent = centeredVoltage / ACS_712_VOLTAGE_PER_AMPS;

    sumSquaredCurrent += instCurrent * instCurrent;

    numSamples++;
  }

  acVoltage.rmsVoltage = sqrt(sumSquaredVoltage / numSamples);
  acVoltage.avgACVoltage = acVoltage.rmsVoltage * acVoltage.voltageDividerGain * AC_VOLTAGE_MEASUREMENT_CORRECTION_FACTOR;

  outputCurrentSense.rmsCurrent = sqrt(sumSquaredCurrent / numSamples) * ACS_712_CALIBRATION_FACTOR;

  // Get DC Voltage
  // Select ADC1 (A1), AVCC as reference
  ADMUX = ADC_INPUT_DC_VOLTAGE;
  delayMicroseconds(5);

  ADCSRA |= (1 << ADSC);
  // Discard result
  while (bit_is_set(ADCSRA, ADSC)) {};
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC)) {};

  float scaledVoltage = (ADC * adcData.adcRes) * (dcVoltage.voltageDividerGain * DC_VOLTAGE_MEASUREMENT_CORRECTION_FACTOR);
  dcVoltage.currentVoltage = roundf(scaledVoltage * 100.0f) / 100.0f;

  #if DEBUG
    Serial.print("Res: ");
    Serial.print(adcData.adcRes * 1000);
    Serial.print(" | ");

    Serial.print("DC: ");
    Serial.print(dcVoltage.currentVoltage, 3);
    Serial.print(" V | ");

    Serial.print("PWMAdj: ");
    Serial.print(spwmControl.waveformScale);
    Serial.print(" | ");

    Serial.print("AC: ");
    Serial.print(acVoltage.avgACVoltage, 3);
    Serial.print("V | ");

    Serial.print("0I: ");
    Serial.print(outputCurrentSense.acs712ZeroCurrentVoltage, 3);
    Serial.print("V | ");

    Serial.print("I: ");
    Serial.print(outputCurrentSense.rmsCurrent, 3);
    Serial.print("A | ");

    Serial.print(upsData.shutdown);
    Serial.print(" | ");
    Serial.print(upsData.switchToAC);
    Serial.print(" | ");
    Serial.print(upsData.ocpStart);
    Serial.print(" | ");

Serial.println("");
  #endif

  // Power handling
  now = millis64();
  upsData.powerOut = now > upsData.lastChangeTime + POWER_CONFIRMATION_DELAY;

  // Logic when AC input is connected
  if (upsData.powerOut) {
    upsData.lastSwitchedToDCOffset = now + DELAY_MS_BEFORE_RELAY_TOGGLES_FROM_DC_TO_DC;
  }

  upsData.switchToAC = now > upsData.lastSwitchedToDCOffset;

  if (upsData.switchToAC) {
    digitalWrite(3, HIGH);
    Buzzer::shutdown(1);
    LED::shutdown();
    upsData.acSwitchedLastAt = now;
    upsData.shutdown = false;
  } else {
    startInverter();
  }
}

ISR(TIMER1_OVF_vect) {
  // Safely disable PWM outputs and set duty to zero to prevent MOSFET shoot-through when switching to AC
  if (upsData.switchToAC || upsData.shutdown || upsData.ocpStart) {
    // Disconnect OC1A and OC1B
    TCCR1A &= ~(spwmControl.output_a_on_mask | spwmControl.output_b_on_mask);

    // Set duty to zero
    OCR1A = OCR1B = 0;

    // Make sure output isn't floating and pull them low
    PORTB &= ~((1 << PB1) | (1 << PB2));

    return;
  }

  // Delay by one period because the high time loaded into OCR1A:B values are buffered but can be disconnected immediately by TCCR1A
  if (spwmControl.pwmToggleDelay) {
    // Ignore 1µs consumed
    // Disconnect & toggle reconnect of compare output A and B
    spwmControl.TCCR1AValue ^= (spwmControl.output_a_on_mask | spwmControl.output_b_on_mask);
    TCCR1A = spwmControl.TCCR1AValue;

    spwmControl.pwmToggleDelay = false;
  }

  if (spwmControl.pwmStepIndex >= spwmControl.half_sinDivisions) {
    // 1 µs
    spwmControl.pwmStepIndex = 0;
    spwmControl.pwmToggleDelay = true;
    // Skip first cycle
    OCR1A = OCR1B = 0;
    // Eat up 9µs. 1µs = 16 cycles. 9µs = 9 * 16 = 144 cycles
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");
    __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\t");
  } else {
    // Takes 10µs
    uint32_t pwm = (spwmControl.lookUp[spwmControl.pwmStepIndex] * spwmControl.waveformScale) >> 7;
    pwm = constrain(pwm, spwmControl.pwmLowClampThreshold, spwmControl.pwmHighClampThreshold);
    OCR1A = OCR1B = pwm;
  }

  spwmControl.pwmStepIndex++;
}

ISR(TIMER2_COMPA_vect) {
  Buzzer::update();
  LED::update();
}

void powerMonitorISR() {
  upsData.lastChangeTime = millis64();
}

// Debugging
#if DEBUG
void uint64ToStr(uint64_t num, char *str) {
  char temp[21];
  uint8_t i = 0;

  if (num == 0) {
    str[0] = '0';
    str[1] = '\0';
    return;
  }

  while (num > 0) {
    // Get last digit
    temp[i++] = (num % 10) + '0';
    num /= 10;
  }

  // Reverse the string
  uint8_t j = 0;
  while (i > 0) {
    str[j++] = temp[--i];
  }

  str[j] = '\0';
}
#endif
