#include <Arduino.h>

// ============================================================
// CODE 1/4: SETTINGS AND PUMP OUTPUT
// ============================================================

// false = Method A (comparator)
// true  = Method B (three-probe ADC)
const bool METHOD_B_ADC = false;

// Keep false during initial sensing tests.
const bool ENABLE_PUMP = false;

// Method B only: set true AFTER actual-water calibration.
const bool B_CALIBRATED = false;

// ESP32 connections
const uint8_t EXC_PIN = 25;
const uint8_t PUMP_PIN = 26;

const uint8_t LOW_PIN = METHOD_B_ADC ? 34 : 32;
const uint8_t HIGH_PIN = 35;  // Method B only

// Timing settings
const uint32_t SAMPLE_MS = 250;
const uint32_t SETTLE_MS = METHOD_B_ADC ? 10 : 20;

// Initial bench-test limit: 2000 milliseconds = 2 seconds.
const uint32_t MAX_ON_MS = 2000;

const uint8_t DRY_READS = 3;
const uint8_t ADC_SAMPLES = 8;

// Provisional Method B thresholds.
// Units are millivolts, NOT raw ADC counts.
// Replace these with your calibrated values.
const uint16_t LOW_DRY_MV = 300;
const uint16_t LOW_WET_MV = 700;

const uint16_t HIGH_DRY_MV = 300;
const uint16_t HIGH_WET_MV = 700;

// Probe states
const uint8_t UNKNOWN = 0;
const uint8_t DRY = 1;
const uint8_t WET = 2;

uint8_t lowState = UNKNOWN;
uint8_t highState = UNKNOWN;

uint16_t lowMv = 0;
uint16_t highMv = 0;

bool pumpRequested = false;
bool fault = false;

const char *faultReason = "none";

uint8_t dryCount = 0;

uint32_t pumpStarted = 0;
uint32_t lastSample = 0;

bool driveAllowed() {
  return ENABLE_PUMP && (!METHOD_B_ADC || B_CALIBRATED);
}

void setPump(bool wanted) {
  if (fault) {
    wanted = false;
  }

  if (wanted && !pumpRequested) {
    pumpStarted = millis();
  }

  pumpRequested = wanted;

  digitalWrite(
    PUMP_PIN,
    driveAllowed() && wanted ? HIGH : LOW
  );
}

// ============================================================
// CODE 2/4: SENSING AND DECISIONS
// ============================================================

void trip(const char *reason) {
  fault = true;
  faultReason = reason;
  setPump(false);
}

void checkTimeout() {
  if (driveAllowed() &&
      pumpRequested &&
      uint32_t(millis() - pumpStarted) >= MAX_ON_MS) {
    trip("fill timeout");
  }
}

uint8_t classify(
  uint16_t mv,
  uint8_t previous,
  uint16_t dryLimit,
  uint16_t wetLimit
) {
  if (mv >= wetLimit) {
    return WET;
  }

  if (mv <= dryLimit) {
    return DRY;
  }

  // Between thresholds: retain the previous state.
  return previous;
}

void readProbes() {
  digitalWrite(EXC_PIN, HIGH);
  delay(SETTLE_MS);

  if (METHOD_B_ADC) {
    uint32_t sumLow = 0;
    uint32_t sumHigh = 0;

    for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
      sumLow += analogReadMilliVolts(LOW_PIN);
      sumHigh += analogReadMilliVolts(HIGH_PIN);
    }

    lowMv = sumLow / ADC_SAMPLES;
    highMv = sumHigh / ADC_SAMPLES;

  } else {
    lowState = digitalRead(LOW_PIN) == LOW ? WET : DRY;
    highState = lowState;
  }

  // End the excitation pulse before processing.
  digitalWrite(EXC_PIN, LOW);

  if (METHOD_B_ADC) {
    lowState = classify(
      lowMv,
      lowState,
      LOW_DRY_MV,
      LOW_WET_MV
    );

    highState = classify(
      highMv,
      highState,
      HIGH_DRY_MV,
      HIGH_WET_MV
    );
  }
}

// ============================================================
// CODE 3/4: REPORTING AND STARTUP
// ============================================================

const char *stateText(uint8_t s) {
  return s == WET ? "WET" : (s == DRY ? "DRY" : "UNKNOWN");
}

void report() {
  if (METHOD_B_ADC) {
    Serial.print("LOW_mV=");
    Serial.print(lowMv);

    Serial.print(" HIGH_mV=");
    Serial.print(highMv);

    Serial.print(" LOWER=");
    Serial.print(stateText(lowState));

    Serial.print(" UPPER=");
    Serial.print(stateText(highState));

  } else {
    Serial.print("SENSE=");
    Serial.print(stateText(lowState));
  }

  Serial.print(" request=");
  Serial.print(pumpRequested ? "ON" : "OFF");

  Serial.print(" output=");
  Serial.print(
    driveAllowed() && pumpRequested ? "ON" : "OFF"
  );

  Serial.print(" fault=");
  Serial.println(faultReason);
}

void setup() {
  digitalWrite(PUMP_PIN, LOW);
  pinMode(PUMP_PIN, OUTPUT);

  digitalWrite(EXC_PIN, LOW);
  pinMode(EXC_PIN, OUTPUT);

  pinMode(LOW_PIN, INPUT);

  Serial.begin(115200);

  if (METHOD_B_ADC) {
    pinMode(HIGH_PIN, INPUT);

    analogReadResolution(12);

    analogSetPinAttenuation(LOW_PIN, ADC_11db);
    analogSetPinAttenuation(HIGH_PIN, ADC_11db);

    if (LOW_DRY_MV >= LOW_WET_MV ||
        HIGH_DRY_MV >= HIGH_WET_MV) {
      trip("invalid thresholds");
    }
  }

  setPump(false);

  Serial.println("Starting OFF; see manual revision 3");

  if (METHOD_B_ADC && !B_CALIBRATED) {
    Serial.println("B pump locked: calibrate before enabling");
  }
}

// ============================================================
// CODE 4/4: THE REFILL LOOP
// ============================================================

void loop() {
  checkTimeout();

  uint32_t now = millis();

  if (uint32_t(now - lastSample) < SAMPLE_MS) {
    return;
  }

  lastSample = now;

  readProbes();
  checkTimeout();

  if (!fault) {
    if (highState == WET) {
      dryCount = 0;

      // Stop first, regardless of the lower probe reading.
      setPump(false);

      if (METHOD_B_ADC && lowState == DRY) {
        trip("upper wet but lower dry");
      }

    } else if (lowState == UNKNOWN || highState == UNKNOWN) {
      dryCount = 0;

      // Ambiguous at startup: do not fill.
      setPump(false);

    } else if (lowState == DRY && highState == DRY) {
      if (dryCount < DRY_READS) {
        ++dryCount;
      }

      if (dryCount >= DRY_READS) {
        setPump(true);
      }

    } else {
      dryCount = 0;

      // Method B:
      // LOWER wet and UPPER dry -> keep the existing pump state.
    }
  }

  report();
}
