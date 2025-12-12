#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// ----- OLED -----
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ----- Accelerometer (LIS3DH) -----
#define LIS3DH_ADDR 0x18
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// ----- Pins -----
#define PIEZO_PIN 12
#define LED_PIN   27

// ----- Thresholds -----
#define AXIS_G_MIN  9.0f
#define AXIS_G_MAX 11.5f

// ----- Axis-specific countdown durations (seconds) -----
//  -Y →  5 s  (bottom)
//  +X → 10 s  (right)
//  +Y → 15 s  (top)
//  -X → 30 s  (left)
#define COUNT_NEG_Y_SEC  5
#define COUNT_POS_X_SEC 10
#define COUNT_POS_Y_SEC 15
#define COUNT_NEG_X_SEC 30

// ----- Alarm timing -----
#define ALARM_TOGGLE_MS     1000   // LED/buzzer toggle every 1s

// Axis-change interrupt buffer
#define AXIS_CHANGE_BUFFER_MS 500  // 0.5 second

// ----- States -----
enum State {
  STATE_MENU = 0,
  STATE_COUNTDOWN,
  STATE_ALARM
};

State currentState = STATE_MENU;

// Which face is active
enum Face {
  FACE_NONE = 0,
  FACE_NEG_Y,
  FACE_POS_X,
  FACE_POS_Y,
  FACE_NEG_X
};

Face activeFace = FACE_NONE;

// Countdown state
int countdownTotalSec = 0;
int countdownRemainingSec = 0;
char countdownAxisLabel = '?';
unsigned long lastCountdownTick = 0;
uint8_t countdownRotation = 0;   // 0..3 for display.setRotation()

// Alarm state
unsigned long lastAlarmToggleMs = 0;
bool alarmOutputState = false;

// Axis change tracking
bool axisChangePending = false;
unsigned long axisChangeStartMs = 0;

// Forward declarations
void goToMenu();
void drawMenu();
void drawCountdownScreen();
void startCountdown(int seconds, char axisLabel, uint8_t rotationIndex, Face face);
void updateCountdown();
void startAlarm();
void updateAlarm();

// Face detection
Face detectFace(float x, float y, float z) {
  float ax = fabsf(x);
  float ay = fabsf(y);
  float az = fabsf(z);

  // Pick the dominant axis
  if (ay >= ax && ay >= az && ay >= AXIS_G_MIN && ay <= AXIS_G_MAX) {
    return (y > 0) ? FACE_POS_Y : FACE_NEG_Y;
  } else if (ax >= ay && ax >= az && ax >= AXIS_G_MIN && ax <= AXIS_G_MAX) {
    return (x > 0) ? FACE_POS_X : FACE_NEG_X;
  } else if (az >= AXIS_G_MIN && az <= AXIS_G_MAX) {
    // Z-up (or Z-down) = menu / neutral → treat as no face
    return FACE_NONE;
  } else {
    return FACE_NONE;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // I2C for OLED + LIS3DH
  Wire.begin(23, 22); // SDA, SCL for Feather ESP32

  // ---- Init OLED ----
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
    while (1) delay(10);
  }

  display.clearDisplay();
  display.display();

  // ---- Init LIS3DH ----
  if (!lis.begin(LIS3DH_ADDR)) {
    Serial.println("Could not find LIS3DH!");
    while (1) delay(10);
  }

  lis.setRange(LIS3DH_RANGE_2_G);
  lis.setDataRate(LIS3DH_DATARATE_50_HZ);

  // ---- Outputs ----
  pinMode(PIEZO_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(PIEZO_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Serial.println("System Ready.");

  // Start at main menu
  goToMenu();
}

void loop() {
  // Read accelerometer
  sensors_event_t event;
  lis.getEvent(&event);

  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;

  Face currentFace = detectFace(x, y, z);
  unsigned long now = millis();

  // Handle axis-change interrupt when in countdown or alarm
  if (currentState == STATE_COUNTDOWN || currentState == STATE_ALARM) {
    if (currentFace == activeFace && currentFace != FACE_NONE) {
      // Stable on original face → cancel any pending interrupt
      axisChangePending = false;
    } else {
      // Face changed (or went to neutral / Z-up)
      if (!axisChangePending) {
        axisChangePending = true;
        axisChangeStartMs = now;
      } else if (now - axisChangeStartMs >= AXIS_CHANGE_BUFFER_MS) {
        // Interrupt after 0.5s stable on a different orientation
        goToMenu();
        return;
      }
    }
  } else {
    // In MENU: no axis-change tracking
    axisChangePending = false;
    activeFace = FACE_NONE;
  }

  switch (currentState) {
    case STATE_MENU: {
      // Choose timer based on face
      //  -Y →  5s (rot 0)
      //  +X → 10s (rot 3)
      //  +Y → 15s (rot 2)
      //  -X → 30s (rot 1)
       if (currentFace == FACE_NEG_Y) {
    startCountdown(COUNT_NEG_Y_SEC, 'Y', 0, FACE_NEG_Y);
  } else if (currentFace == FACE_POS_X) {
    startCountdown(COUNT_NEG_X_SEC, 'X', 1, FACE_POS_X);
  } else if (currentFace == FACE_POS_Y) {
    startCountdown(COUNT_POS_Y_SEC, 'Y', 2, FACE_POS_Y);
  } else if (currentFace == FACE_NEG_X) {
    startCountdown(COUNT_POS_X_SEC, 'X', 3, FACE_NEG_X);
  }
  break;
    }

    case STATE_COUNTDOWN: {
      updateCountdown();
      break;
    }

    case STATE_ALARM: {
      updateAlarm();
      break;
    }
  }

  delay(50);
}

// ---------------------------------------------------------
// Menu logic
// ---------------------------------------------------------
void goToMenu() {
  currentState = STATE_MENU;
  activeFace   = FACE_NONE;
  axisChangePending = false;

  // Turn off outputs just in case
  digitalWrite(LED_PIN, LOW);
  digitalWrite(PIEZO_PIN, LOW);
  noTone(PIEZO_PIN);

  // Menu is always drawn upright
  display.setRotation(0);
  drawMenu();
}


void drawMenu() {
  display.clearDisplay();
  display.setRotation(0);

  // Your preferred centered-ish "Choose"
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(48, 28);
  display.println("Choose");

  // Bottom: 5s  (-Y)
  display.setCursor(57, 54);
  display.print("5s");
  display.setCursor(60, 44);
  display.print("v");

  // Right: 10s (+X)
  display.setCursor(104, 28);
  display.print("10s");
  display.setCursor(96, 30);
  display.print(">");

  // Top: 15s (+Y)
  display.setCursor(54, 4);
  display.print("15s");
  display.setCursor(60, 14);
  display.print("^");

  // Left: 30s (-X)
  display.setCursor(4, 28);
  display.print("30s");
  display.setCursor(24, 30);
  display.print("<");

  display.display();
}

// ---------------------------------------------------------
// Countdown drawing (handles X vs Y orientations)
// ---------------------------------------------------------
void drawCountdownScreen() {
  display.clearDisplay();
  display.setRotation(countdownRotation);
  display.setTextColor(SSD1306_WHITE);

  char buf[8];
  snprintf(buf, sizeof(buf), "%d", countdownRemainingSec);

  // Y faces (rot 0 or 2): big number near center
  if (countdownRotation == 0 || countdownRotation == 2) {
    display.setTextSize(3);
    display.setCursor(40, 24);
    display.println(buf);
  }
  // X faces (rot 1 or 3): big centered number in 64px-wide orientation
  else {
    display.setTextSize(3);

    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);

    int16_t x = (display.width()  - w) / 2;
    int16_t y = (display.height() - h) / 2;

    display.setCursor(x, y);
    display.println(buf);
  }

  display.display();
}


// ---------------------------------------------------------
// Countdown logic (non-blocking)
// ---------------------------------------------------------
void startCountdown(int seconds, char axisLabel, uint8_t rotationIndex, Face face) {
  countdownTotalSec      = seconds;
  countdownRemainingSec  = seconds;
  countdownAxisLabel     = axisLabel;
  countdownRotation      = rotationIndex;
  lastCountdownTick      = millis();
  currentState           = STATE_COUNTDOWN;
  activeFace             = face;
  axisChangePending      = false;

  drawCountdownScreen();
}

void updateCountdown() {
  if (currentState != STATE_COUNTDOWN) return;

  unsigned long now = millis();

  if (now - lastCountdownTick >= 1000) {
    lastCountdownTick = now;

    if (countdownRemainingSec > 0) {
      countdownRemainingSec--;
      drawCountdownScreen();
    } else {
      startAlarm();
    }
  }
}

// ---------------------------------------------------------
// Alarm logic (non-blocking, runs until axis change)
// ---------------------------------------------------------

// ----- Alarm melody -----
const int melody[] = {
  659, 587, 370, 415,
  554, 494, 294, 330,
  494, 440, 277, 330,
  440
};

// Note durations (ms)
const int noteDurations[] = {
  150, 150, 300, 300,
  150, 150, 300, 300,
  150, 150, 300, 300,
  600
};

const int NUM_NOTES = sizeof(melody) / sizeof(melody[0]);


int currentNoteIndex = 0;
unsigned long noteStartMs = 0;


void startAlarm() {
  currentState        = STATE_ALARM;
  lastAlarmToggleMs   = millis();
  alarmOutputState    = false;

  // init melody
  currentNoteIndex = 0;
  noteStartMs      = millis();

  digitalWrite(LED_PIN, LOW);
  digitalWrite(PIEZO_PIN, LOW);

  // start first tone (if not a rest)
  if (melody[currentNoteIndex] > 0) {
    tone(PIEZO_PIN, melody[currentNoteIndex]);
  } else {
    noTone(PIEZO_PIN);
  }

  display.clearDisplay();
  display.setRotation(countdownRotation);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  // Y faces (rot 0 or 2): two-line "Time's / Up!"
  if (countdownRotation == 0 || countdownRotation == 2) {
    display.setCursor(5, 20);
    display.println("Time's");
    display.setCursor(20, 40);
    display.println("Up!");
  }
  // X faces (rot 1 or 3): single-line "Time is Up!"
  else {
    const char* msg = "Time is Up!";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);
    int16_t x = (display.width()  - w) / 2;
    int16_t y = (display.height() - h) / 2;
    display.setCursor(x, y);
    display.println(msg);
  }

  display.display();
}


void updateAlarm() {
  if (currentState != STATE_ALARM) return;

  unsigned long now = millis();

  // 1) Blink LED at 1 Hz using ALARM_TOGGLE_MS
  if (now - lastAlarmToggleMs >= ALARM_TOGGLE_MS) {
    lastAlarmToggleMs = now;
    alarmOutputState = !alarmOutputState;
    digitalWrite(LED_PIN, alarmOutputState);
  }

  // 2) Step through melody notes
  if (now - noteStartMs >= (unsigned long)noteDurations[currentNoteIndex]) {
    // move to next note
    currentNoteIndex++;
    if (currentNoteIndex >= NUM_NOTES) {
      currentNoteIndex = 0; // loop melody
    }
    noteStartMs = now;

    int freq = melody[currentNoteIndex];
    if (freq > 0) {
      tone(PIEZO_PIN, freq);
    } else {
      noTone(PIEZO_PIN);  // rest
    }
  }
}
