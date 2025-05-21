class LED {
public:
  static void begin(uint8_t pin);
  static void flash(uint8_t count, uint16_t duration, uint16_t interval, uint16_t repeatDelay);
  static void update();
  static void shutdown();
  static void resume();
  static void startFlash();
  static void stopFlash();
  static bool isShutdown();

private:
  static uint8_t _pin;
  static bool inFlashSequence;
  static bool ledOn;
  static uint32_t nextToggleTime;
  static uint32_t lastSequenceTime;
  static uint16_t flashIndex;
  static uint16_t flashCount;
  static uint16_t flashDuration;
  static uint16_t flashInterval;
  static uint32_t buzzRepeatDelay;
  static bool pendingRequest;
  static bool stopped;
  static volatile uint8_t* _buzzerPort;
  static uint8_t _buzzerBit;
  static volatile uint8_t* _ledPort;
  static uint8_t _ledBit;
};

uint8_t LED::_pin = 0;
bool LED::inFlashSequence = false;
bool LED::ledOn = false;
uint32_t LED::nextToggleTime = 0;
uint32_t LED::lastSequenceTime = 0;
uint16_t LED::flashIndex = 0;
uint16_t LED::flashCount = 0;
uint16_t LED::flashDuration = 0;
uint16_t LED::flashInterval = 0;
uint32_t LED::buzzRepeatDelay = 5000;
bool LED::pendingRequest = false;
bool LED::stopped = false;
volatile uint8_t* LED::_ledPort;
uint8_t LED::_ledBit;

void LED::begin(uint8_t pin) {
  _pin = pin;

  _ledPort = portOutputRegister(digitalPinToPort(_pin));
  _ledBit = digitalPinToBitMask(_pin);

  pinMode(_pin, OUTPUT);

  // Set LED LOW
  *_ledPort &= ~_ledBit;
}

void LED::flash(uint8_t count, uint16_t duration, uint16_t interval, uint16_t repeatDelay) {
  // Save the request parameters even if it's not time to trigger yet
  flashCount = count;
  flashDuration = duration;
  flashInterval = interval;
  buzzRepeatDelay = repeatDelay;
  pendingRequest = true;
}

void LED::update() {
  uint32_t now = millis();

  // Handle pending beep request, only if not in sequence and cooldown is over
  if (pendingRequest && !inFlashSequence && (now - lastSequenceTime >= buzzRepeatDelay)) {
    flashIndex = 0;
    ledOn = true;
    startFlash();
    nextToggleTime = now + flashDuration;
    inFlashSequence = true;
    pendingRequest = false;
  }

  if (inFlashSequence && now >= nextToggleTime) {
    if (ledOn) {
      stopFlash();
      ledOn = false;
      nextToggleTime = now + flashInterval;
    } else {
      flashIndex++;
      if (flashIndex < flashCount) {
        startFlash();
        ledOn = true;
        nextToggleTime = now + flashDuration;
      } else {
        inFlashSequence = false;
        stopFlash();
        lastSequenceTime = now;
      }
    }
  }
}

void LED::shutdown() {
  stopped = true;
  LED::stopFlash();
}

bool LED::isShutdown() {
  return stopped;
}

void LED::resume() {
  stopped = false;
}

void LED::startFlash() {
  if (!stopped) {
    *_ledPort |= _ledBit;
  }
}

void LED::stopFlash() {
  *_ledPort &= ~_ledBit;
}
