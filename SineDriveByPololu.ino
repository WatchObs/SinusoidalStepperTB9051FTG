/*
  Hybrid stepper (float integration) with loss-of-pulses safety and 2 Hz logging
  - External STEP on pin 21 used only for measurement (period + count)
  - ISR integrates published host rate (indices per ISR) into a float phase accumulator
  - If no STEP edges seen for LOSS_TIMEOUT_MS, published rate is forced to zero
  - PIN_RATE_PWM (pin 11) configured once at 220 Hz and left untouched
  - ENABLE and DIR forced in software via SOFTWARE_ENABLE / SOFTWARE_DIR_FORWARD
  - Debug logging prints countRate, periodRate, chosenRate, source, usSinceLastEdge, phaseStepPerIsr every 500 ms
*/

#include <Arduino.h>
#include <IntervalTimer.h>
#include <stdint.h>
#include <math.h>

// --- Configuration ---
const bool SOFTWARE_ENABLE = true;         // force driver enable
const bool SOFTWARE_DIR_FORWARD = true;    // force direction
const bool ENABLE_DEBUG = true;            // enable 2 Hz serial debug prints

// Loss-of-pulses timeout (milliseconds)
const unsigned long LOSS_TIMEOUT_MS = 1000UL; // 1 second

// Pins
const int PIN_AEN   = 5;
const int PIN_AENB  = 4;
const int PIN_APWM1 = 2;
const int PIN_APWM2 = 3;
const int PIN_BEN   = 9;
const int PIN_BENB  = 8;
const int PIN_BPWM1 = 7;
const int PIN_BPWM2 = 6;
const int PIN_LED   = 13;

const int PIN_OCM1 = A1;
const int PIN_OCM2 = A2;

const int PIN_HOST_ENABLE = 22;
const int PIN_HOST_DIR    = 23;
const int PIN_HOST_STEP   = 21; // external STEP input (measurement only)

const int PIN_RATE_PWM = 11;    // fixed hardware PWM test source (220 Hz)
const uint32_t RATE_PWM_FREQ_HZ = 220;
const uint32_t RATE_PWM_DUTY_16 = 32768; // 50% duty (0..65535)

const int PIN_ISR_TOGGLE = 10;  // scope probe

// Waveform / motor geometry
const int angles = 1024;            // table entries per electrical cycle
const int cycles_per_turn = 50;     // electrical cycles per mechanical revolution

// ISR timing
const uint32_t ISR_FREQ_HZ = 20000UL; // ISR ticks per second
const uint32_t ISR_PERIOD_US = 1000000UL / ISR_FREQ_HZ;

// Measurement parameters
const unsigned long windowMs = 10UL;        // counting window for fast pulses
const double periodThresholdUs = 200.0;     // if single-edge period >= this, use period method

// ADC
const int ADC_BITS = 12;

// Deadtime
const uint8_t DEAD_CYCLES_ON_FLIP = 0;

// --- Sine table ---
static int32_t tbl[angles];
static void setupSineTable() {
  for (int n = 0; n < angles; ++n) {
    double a = (double)n * 2.0 * M_PI / (double)angles;
    tbl[n] = (int32_t)round(sin(a) * 65535.0);
  }
}

// Interpolated lookup using float index
static inline int32_t lookupInterpolatedFloat(float idx) {
  if (idx >= (float)angles) idx -= floorf(idx / (float)angles) * (float)angles;
  if (idx < 0.0f) idx += ceilf(-idx / (float)angles) * (float)angles;
  int i = (int)floorf(idx);
  float frac = idx - (float)i;
  int j = (i + 1 >= angles) ? 0 : i + 1;
  int32_t v0 = tbl[i];
  int32_t v1 = tbl[j];
  float interp = (float)v0 + ((float)(v1 - v0)) * frac;
  return (int32_t)roundf(interp);
}

// --- Shared state (volatile for ISR/main) ---
volatile float phaseA_f = 0.0f;             // floating index into table [0..angles)
volatile float phaseStepPerIsr_f = 0.0f;    // indices to advance per ISR tick (published by loop)
volatile float latestRatePps_f = 0.0f;      // latest chosen rate in pulses/sec (for diagnostics)
volatile int8_t dirSign = 1;                // +1 forward, -1 reverse

// PWM outputs (written by ISR)
volatile uint16_t pwmA_pos = 0, pwmA_neg = 0, pwmB_pos = 0, pwmB_neg = 0;
volatile int8_t lastSignA = 0, lastSignB = 0;
volatile uint8_t deadCyclesRemaining = 0;

// Measurement variables (updated in ISR)
volatile uint32_t lastEdgeCycles = 0;       // DWT cycles of last edge
volatile uint32_t lastPeriodCycles = 0;     // cycles between last two edges
volatile uint32_t pulseCountWindow = 0;     // incremented per edge (cleared by loop)

// Diagnostics / housekeeping
volatile uint32_t stepsSinceLastSample = 0;
volatile uint16_t latestRaw1 = 0, latestRaw2 = 0;
volatile uint32_t isrHeartbeat = 0;

// IntervalTimer
IntervalTimer stepTimer;

// --- DWT cycle counter (raw registers) ---
static inline void enableCycleCounter() {
  *(volatile uint32_t *)0xE000EDFC |= (1UL << 24); // DEMCR TRCENA
  *(volatile uint32_t *)0xE0001004 = 0;            // DWT_CYCCNT = 0
  *(volatile uint32_t *)0xE0001000 |= 1UL;         // DWT_CTRL CYCCNTENA
}
static inline uint32_t readCycleCounter() {
  return *(volatile uint32_t *)0xE0001004;
}

// --- API: publish host microsteps/sec (host unit = pulses/sec where 1 pulse = 1 table index) ---
void setMicrostepsPerSec(double microstepsPerSec) {
  if (microstepsPerSec < 0.0) microstepsPerSec = -microstepsPerSec;
  double stepPerIsr = microstepsPerSec / (double)ISR_FREQ_HZ; // indices per ISR
  noInterrupts();
  phaseStepPerIsr_f = (float)stepPerIsr;
  latestRatePps_f = (float)microstepsPerSec;
  interrupts();
}

// --- ISR: measurement + integration (no per-edge phase changes) ---
void stepISR() {
  digitalWriteFast(PIN_ISR_TOGGLE, HIGH);
  isrHeartbeat++;

  // 1) Measurement: read STEP pin and timestamp rising edges
  uint8_t stepState = digitalReadFast(PIN_HOST_STEP) ? 1 : 0;
  static uint8_t lastStepInputState = 0;
  bool stepRising = (lastStepInputState == 0 && stepState == 1);
  lastStepInputState = stepState;

  if (stepRising) {
    uint32_t nowCycles = readCycleCounter();
    uint32_t prev = lastEdgeCycles;
    uint32_t period = (nowCycles - prev); // wrap-safe
    if (prev != 0) lastPeriodCycles = period;
    lastEdgeCycles = nowCycles;
    pulseCountWindow++; // counted by loop() for count-based estimator
  }

  // 2) Phase integration from published host rate (float read is atomic on 32-bit)
  float inc = phaseStepPerIsr_f;
  if (inc != 0.0f) {
    phaseA_f += (float)dirSign * inc;
    if (phaseA_f >= (float)angles) phaseA_f -= (float)angles * floorf(phaseA_f / (float)angles);
    else if (phaseA_f < 0.0f) phaseA_f += (float)angles * ceilf(-phaseA_f / (float)angles);
  }

  // 3) Compute channel B (quarter period ahead) and lookup
  float phaseB_f = phaseA_f + ((float)angles / 4.0f);
  if (phaseB_f >= (float)angles) phaseB_f -= (float)angles;

  // Deadtime handling (disabled by default)
  if (deadCyclesRemaining > 0) {
    pwmA_pos = pwmA_neg = pwmB_pos = pwmB_neg = 0;
    deadCyclesRemaining--;
  } else {
    int32_t valA = lookupInterpolatedFloat(phaseA_f);
    int32_t valB = lookupInterpolatedFloat(phaseB_f);

    int8_t signA = (valA < 0) ? -1 : ((valA > 0) ? 1 : 0);
    int8_t signB = (valB < 0) ? -1 : ((valB > 0) ? 1 : 0);

    if ((lastSignA != 0 && signA != lastSignA) || (lastSignB != 0 && signB != lastSignB)) {
      if (DEAD_CYCLES_ON_FLIP > 0) {
        pwmA_pos = pwmA_neg = pwmB_pos = pwmB_neg = 0;
        deadCyclesRemaining = DEAD_CYCLES_ON_FLIP;
        lastSignA = signA; lastSignB = signB;
      } else {
        if (valA < 0) { pwmA_pos = 0; pwmA_neg = (uint16_t)(-valA); } else { pwmA_pos = (uint16_t)valA; pwmA_neg = 0; }
        if (valB < 0) { pwmB_pos = 0; pwmB_neg = (uint16_t)(-valB); } else { pwmB_pos = (uint16_t)valB; pwmB_neg = 0; }
        lastSignA = signA; lastSignB = signB;
      }
    } else {
      if (valA < 0) { pwmA_pos = 0; pwmA_neg = (uint16_t)(-valA); } else { pwmA_pos = (uint16_t)valA; pwmA_neg = 0; }
      if (valB < 0) { pwmB_pos = 0; pwmB_neg = (uint16_t)(-valB); } else { pwmB_pos = (uint16_t)valB; pwmB_neg = 0; }
      lastSignA = signA; lastSignB = signB;
    }
  }

  // 4) Write PWM outputs
  analogWrite(PIN_APWM1, pwmA_pos);
  analogWrite(PIN_APWM2, pwmA_neg);
  analogWrite(PIN_BPWM1, pwmB_pos);
  analogWrite(PIN_BPWM2, pwmB_neg);

  // 5) ADC reads and housekeeping
  latestRaw1 = (uint16_t)analogRead(PIN_OCM1);
  latestRaw2 = (uint16_t)analogRead(PIN_OCM2);
  stepsSinceLastSample++;
  digitalWriteFast(PIN_ISR_TOGGLE, LOW);
}

// --- Loop: choose period vs count estimator, detect loss-of-pulses, publish chosen rate ---
// Also prints debug info at 2 Hz when ENABLE_DEBUG is true.
void publishChosenRateFromMeasurement(double chosenRatePps) {
  setMicrostepsPerSec(chosenRatePps);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println("Hybrid stepper (float integration) - STEP on pin 21; pin 11 PWM @220Hz; debug=2Hz");

  // pins
  pinMode(PIN_AEN, OUTPUT); pinMode(PIN_AENB, OUTPUT);
  pinMode(PIN_APWM1, OUTPUT); pinMode(PIN_APWM2, OUTPUT);
  pinMode(PIN_BEN, OUTPUT); pinMode(PIN_BENB, OUTPUT);
  pinMode(PIN_BPWM1, OUTPUT); pinMode(PIN_BPWM2, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_HOST_ENABLE, INPUT_PULLUP);
  pinMode(PIN_HOST_DIR, INPUT_PULLUP);
  pinMode(PIN_HOST_STEP, INPUT_PULLUP); // STEP input (measurement only)

  pinMode(PIN_ISR_TOGGLE, OUTPUT);
  digitalWriteFast(PIN_ISR_TOGGLE, LOW);

  // Configure fixed hardware PWM on PIN_RATE_PWM once and never touch it again
  pinMode(PIN_RATE_PWM, OUTPUT);
  analogWriteResolution(16); // 0..65535
  analogWriteFrequency(PIN_RATE_PWM, RATE_PWM_FREQ_HZ);
  analogWrite(PIN_RATE_PWM, (int)RATE_PWM_DUTY_16); // set and leave

  // Configure other PWM outputs
  analogWriteFrequency(PIN_APWM1, 2000);
  analogWriteFrequency(PIN_APWM2, 2000);
  analogWriteFrequency(PIN_BPWM1, 2000);
  analogWriteFrequency(PIN_BPWM2, 2000);

  analogReadResolution(ADC_BITS);

  analogWrite(PIN_APWM1, 0); analogWrite(PIN_APWM2, 0);
  analogWrite(PIN_BPWM1, 0); analogWrite(PIN_BPWM2, 0);

  setupSineTable();

  // Apply software-forced enable pins state
  if (SOFTWARE_ENABLE) {
    digitalWrite(PIN_AENB, LOW); digitalWrite(PIN_AEN, HIGH);
    digitalWrite(PIN_BENB, LOW); digitalWrite(PIN_BEN, HIGH);
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_AEN, LOW); digitalWrite(PIN_AENB, HIGH);
    digitalWrite(PIN_BEN, LOW); digitalWrite(PIN_BENB, HIGH);
    digitalWrite(PIN_LED, LOW);
  }

  // Direction sign
  dirSign = SOFTWARE_DIR_FORWARD ? 1 : -1;

  // Start with zero host rate (no continuous stepping until host publishes)
  setMicrostepsPerSec(0.0);

  // enable DWT cycle counter
  enableCycleCounter();

  // start ISR timer
  stepTimer.begin(stepISR, ISR_PERIOD_US);
}

void loop() {
  static unsigned long lastWindowMs = 0;
  static unsigned long lastDebugMs = 0;
  unsigned long now = millis();

  // sample every windowMs
  if (now - lastWindowMs >= windowMs) {
    // atomically read and clear pulseCountWindow
    uint32_t count;
    uint32_t periodCycles;
    noInterrupts();
    count = pulseCountWindow;
    pulseCountWindow = 0;
    periodCycles = lastPeriodCycles; // may be 0 if not yet measured
    interrupts();

    // compute count-based rate (pulses/sec)
    double countRate = (double)count / ((double)windowMs / 1000.0);

    // compute period-based rate if available
    double periodRate = 0.0;
    bool havePeriod = (periodCycles != 0);
    double periodUs = 0.0;
    if (havePeriod) {
      double cpuHz = (double)F_CPU;
      periodUs = (double)periodCycles * 1e6 / cpuHz;
      if (periodUs > 0.0) periodRate = 1.0 / (periodUs * 1e-6);
    }

    // Check for loss of pulses: time since lastEdgeCycles
    uint32_t nowCycles = readCycleCounter();
    uint32_t lastEdge = lastEdgeCycles; // volatile read
    uint32_t cyclesSinceLastEdge = (lastEdge == 0) ? 0 : (nowCycles - lastEdge);
    double usSinceLastEdge = (double)cyclesSinceLastEdge * 1e6 / (double)F_CPU;

    bool pulsesLost = (lastEdge == 0) || (usSinceLastEdge >= (double)LOSS_TIMEOUT_MS * 1000.0);

    // choose method or force stop if pulses lost
    double chosenRate = 0.0;
    bool fromPeriod = false;
    if (pulsesLost) {
      // Force stop: clear period so stale value isn't reused
      noInterrupts();
      lastPeriodCycles = 0;
      pulseCountWindow = 0;
      interrupts();
      chosenRate = 0.0;
      fromPeriod = false;

      // publish zero immediately
      publishChosenRateFromMeasurement(0.0);
      if (ENABLE_DEBUG) {
        Serial.print("PULSES LOST: usSinceLastEdge=");
        Serial.print(usSinceLastEdge, 0);
        Serial.println(" -> forcing rate=0");
      }
    } else {
      if (havePeriod && periodUs >= periodThresholdUs) {
        chosenRate = periodRate;
        fromPeriod = true;
      } else {
        chosenRate = countRate;
        fromPeriod = false;
      }

      // publish chosen rate (host pulses/sec -> indices/sec)
      publishChosenRateFromMeasurement(chosenRate);
    }

    lastWindowMs = now;
  }

  // Keep software-forced enable pins consistent
  if (SOFTWARE_ENABLE) {
    digitalWrite(PIN_AENB, LOW); digitalWrite(PIN_AEN, HIGH);
    digitalWrite(PIN_BENB, LOW); digitalWrite(PIN_BEN, HIGH);
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_AEN, LOW); digitalWrite(PIN_AENB, HIGH);
    digitalWrite(PIN_BEN, LOW); digitalWrite(PIN_BENB, HIGH);
    digitalWrite(PIN_LED, LOW);
  }

  // 2 Hz debug printing
  if (ENABLE_DEBUG && (millis() - lastDebugMs >= 500)) {
    // snapshot values atomically
    noInterrupts();
    float pA = phaseA_f;
    float pInc = phaseStepPerIsr_f;
    float lr = latestRatePps_f;
    uint32_t lp = lastPeriodCycles;
    uint32_t lastEdge = lastEdgeCycles;
    interrupts();

    // compute periodRate and countRate for display (recompute from lastPeriodCycles and last window)
    double periodRate = 0.0;
    double periodUs = 0.0;
    if (lp != 0) {
      periodUs = (double)lp * 1e6 / (double)F_CPU;
      if (periodUs > 0.0) periodRate = 1.0 / (periodUs * 1e-6);
    }

    // For countRate display we can't reconstruct the last window count here (it was cleared),
    // so print the latestRatePps_f (what was published) and the instantaneous periodRate.
    double usSinceLastEdge = 0.0;
    if (lastEdge != 0) {
      uint32_t nowCycles = readCycleCounter();
      uint32_t cyclesSinceLastEdge = nowCycles - lastEdge;
      usSinceLastEdge = (double)cyclesSinceLastEdge * 1e6 / (double)F_CPU;
    }

    Serial.print("DBG: publishedRate=");
    Serial.print(lr, 3);
    Serial.print(" pps  perPeriodRate=");
    Serial.print(periodRate, 3);
    Serial.print(" pps  phaseInc=");
    Serial.print(pInc, 6);
    Serial.print(" idx/ISR  phaseA=");
    Serial.print(pA, 3);
    Serial.print("  usSinceLastEdge=");
    Serial.print(usSinceLastEdge, 0);
    Serial.println(" us");

    lastDebugMs = millis();
  }

  delay(1);
}
