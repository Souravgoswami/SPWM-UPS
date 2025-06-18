#ifndef BUZZER_ENABLED
#define BUZZER_ENABLED true
#endif

class Buzzer {
  public:
    static void begin(uint8_t buzzerPin, uint8_t ledPin);
    static void beep(uint8_t count, uint16_t duration, uint16_t interval, uint16_t repeatDelay);
    static void update();
    static void shutdown(bool ledState = 0);
    static void resume();
    static void startBeep();
    static void stopBeep();
    static bool isShutdown();
    static void triggerShutdownAlarm(uint16_t milliSeconds);
    static bool isBusy() { return inBeepSequence; }

  private:
    static uint8_t _buzzerPin;
    static uint8_t _ledPin;
    static bool inBeepSequence;
    static bool buzzerOn;
    static uint32_t nextToggleTime;
    static uint32_t lastSequenceTime;
    static uint16_t beepIndex;
    static uint16_t buzz_count;
    static uint16_t buzz_duration;
    static uint16_t buzz_interval;
    static uint32_t buzzRepeatDelay;
    static bool pendingRequest;
    static bool stopped;
    static bool longBeepActive;
    static uint32_t longBeepEndTick;
    static volatile uint8_t* _buzzerPort;
    static uint8_t _buzzerBit;
    static volatile uint8_t* _ledPort;
    static uint8_t _ledBit;
};

uint8_t Buzzer::_buzzerPin = 0;
uint8_t Buzzer::_ledPin = 0;
bool Buzzer::inBeepSequence = false;
bool Buzzer::buzzerOn = false;
uint32_t Buzzer::nextToggleTime = 0;
uint32_t Buzzer::lastSequenceTime = 0;
uint16_t Buzzer::beepIndex = 0;
uint16_t Buzzer::buzz_count = 0;
uint16_t Buzzer::buzz_duration = 0;
uint16_t Buzzer::buzz_interval = 0;
uint32_t Buzzer::buzzRepeatDelay = 5000;
bool Buzzer::pendingRequest = false;
bool Buzzer::stopped = false;
bool Buzzer::longBeepActive = false;
uint32_t Buzzer::longBeepEndTick = 0;
volatile uint8_t* Buzzer::_buzzerPort;
uint8_t Buzzer::_buzzerBit;
volatile uint8_t* Buzzer::_ledPort;
uint8_t Buzzer::_ledBit;

void Buzzer::begin(uint8_t buzzerPin, uint8_t ledPin) {
  _buzzerPin = buzzerPin;
  _ledPin = ledPin;

  pinMode(_buzzerPin, OUTPUT);
  pinMode(_ledPin, OUTPUT);

  _buzzerPort = portOutputRegister(digitalPinToPort(_buzzerPin));
  _buzzerBit = digitalPinToBitMask(_buzzerPin);
  _ledPort = portOutputRegister(digitalPinToPort(_ledPin));
  _ledBit = digitalPinToBitMask(_ledPin);

  // BUZZER -> LOW
  *_buzzerPort &= ~_buzzerBit;

  // LED -> HIGH
  *_ledPort |= _ledBit;
}

void Buzzer::triggerShutdownAlarm(uint16_t milliSeconds) {
  if (stopped || longBeepActive || inBeepSequence) return;

  longBeepActive = true;
  longBeepEndTick = millis() + milliSeconds;

  startBeep();
}

void Buzzer::beep(uint8_t count, uint16_t duration, uint16_t interval, uint16_t repeatDelay) {
  // Save the request parameters even if it's not time to trigger yet
  buzz_count = count;
  buzz_duration = duration;
  buzz_interval = interval;
  buzzRepeatDelay = repeatDelay;
  pendingRequest = true;
}

void Buzzer::update() {
  uint32_t now = millis();

  // Handle long single beep (non-interrupting)
  if (longBeepActive) {
    if (now >= longBeepEndTick) {
      stopBeep();
      longBeepActive = false;
      shutdown();
    }

    return;
  }

  // Handle pending beep request, only if not in sequence and cooldown is over
  if (pendingRequest && !inBeepSequence && (now - lastSequenceTime >= buzzRepeatDelay)) {
    beepIndex = 0;
    buzzerOn = true;
    startBeep();
    nextToggleTime = now + buzz_duration;
    inBeepSequence = true;
    pendingRequest = false;
  }

  if (inBeepSequence && now >= nextToggleTime) {
    if (buzzerOn) {
      stopBeep();
      buzzerOn = false;
      nextToggleTime = now + buzz_interval;
    } else {
      beepIndex++;
      if (beepIndex < buzz_count) {
        startBeep();
        buzzerOn = true;
        nextToggleTime = now + buzz_duration;
      } else {
        inBeepSequence = false;
        stopBeep();
        lastSequenceTime = now;
      }
    }
  }
}

void Buzzer::shutdown(bool ledState) {

  if (BUZZER_ENABLED) {
    // BUZZER -> LOW
    *_buzzerPort &= ~_buzzerBit;
  }

  // LED -> LOW
  if (ledState) {
    *_ledPort |= _ledBit;
  } else {
    *_ledPort &= ~_ledBit;
  }

  stopped = true;
}

bool Buzzer::isShutdown() {
  return stopped;
}

void Buzzer::resume() {
  if (stopped) {
    // Resume to default state
    // BUZZER -> LOW
    *_buzzerPort &= ~_buzzerBit;

    // LED -> HIGH
    *_ledPort |= _ledBit;

    stopped = false;
  }
}

void Buzzer::startBeep() {
  if (!stopped) {
    if (BUZZER_ENABLED) {
      // Buzzer -> HIGH
      *_buzzerPort |= _buzzerBit;
    }

      // LED -> LOW
    *_ledPort &= ~_ledBit;
  }
}

void Buzzer::stopBeep() {
  if (!stopped) {
    if (BUZZER_ENABLED) {
      // Buzzer -> LOW
    *_buzzerPort &= ~_buzzerBit;
    }

    // LED -> HIGH
  *_ledPort |= _ledBit;
  }
}
