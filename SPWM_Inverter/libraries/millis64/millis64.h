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
