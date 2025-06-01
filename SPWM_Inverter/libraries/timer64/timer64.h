uint64_t millis64() {
  static uint32_t lastMillis = 0;
  static uint64_t overflowMillis = 0;

  uint32_t currentMillis = millis();

  if (currentMillis < lastMillis) {
    // Overflow happened!
    overflowMillis += (uint64_t)1 << 32;
  }

  lastMillis = currentMillis;
  return overflowMillis + currentMillis;
}

uint64_t micros64() {
  static uint32_t lastMicros = 0;
  static uint64_t overflowMicros = 0;

  uint32_t currentMicros = micros();

  if (currentMicros < lastMicros) {
    // Overflow happened (~70 minutes at 4-byte micros())
    overflowMicros += (uint64_t)1 << 32;
  }

  lastMicros = currentMicros;
  return overflowMicros + currentMicros;
}
