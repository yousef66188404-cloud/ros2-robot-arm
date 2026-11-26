/*
 * arm_servo.ino - receives joint angles from the ROS 2 hardware bridge over
 * USB serial and drives hobby servos (MG996R / MG995 / SG90 ...) with PWM.
 *
 * Works on: ESP32 (needs the "ESP32Servo" library), ESP8266 and Arduino Uno/Nano.
 * The board is detected automatically at compile time.
 *
 * Protocol (ASCII, one line per command, 115200 baud):
 *     A,<deg1>,<deg2>\n        e.g.  A,90,45\n
 *   - <degN> is an integer 0..180 for servo N (values are clamped here too).
 *   - Board answers "READY" once after reset and "ERR" for a malformed line.
 *
 * Wiring (signal pins; servo power ALWAYS comes from an external supply):
 *     ESP32   : servo 1 -> GPIO18, servo 2 -> GPIO19
 *     ESP8266 : servo 1 -> D1 (GPIO5), servo 2 -> D2 (GPIO4)
 *     Uno/Nano: servo 1 -> D9,  servo 2 -> D10
 *     Servo VCC (red)   -> external 5-6 V supply (+)
 *     Servo GND (brown) -> external supply (-) AND board GND (common ground)
 *
 * To add a servo: raise NUM_SERVOS and add a pin to SERVO_PINS.
 */
#if defined(ESP32)
  #include <ESP32Servo.h>
  #define PIN_SERVO1 18
  #define PIN_SERVO2 19
#elif defined(ESP8266)
  #include <Servo.h>
  #define PIN_SERVO1 5   // D1
  #define PIN_SERVO2 4   // D2
#else
  #include <Servo.h>
  #define PIN_SERVO1 9
  #define PIN_SERVO2 10
#endif

const long BAUD = 115200;
const uint8_t NUM_SERVOS = 2;
const uint8_t SERVO_PINS[NUM_SERVOS] = {PIN_SERVO1, PIN_SERVO2};

// Pulse range in microseconds. 500-2500 gives the full ~180 deg on MG996R/MG995.
// If a servo buzzes at the ends, narrow it (e.g. 600-2400).
const int PULSE_MIN_US = 500;
const int PULSE_MAX_US = 2500;

Servo servos[NUM_SERVOS];
char line[48];
uint8_t len = 0;

void setup() {
  Serial.begin(BAUD);
#if defined(ESP32)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
#endif
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
#if defined(ESP32)
    servos[i].setPeriodHertz(50);  // standard 50 Hz servo signal
#endif
    servos[i].attach(SERVO_PINS[i], PULSE_MIN_US, PULSE_MAX_US);
    servos[i].write(90);           // start at centre
  }
  Serial.println(F("READY"));
}

void handleLine(char *s) {
  if (s[0] != 'A' || s[1] != ',') {
    Serial.println(F("ERR"));
    return;
  }
  char *p = s + 2;
  int angles[NUM_SERVOS];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p) {  // no number found
      Serial.println(F("ERR"));
      return;
    }
    angles[i] = constrain(v, 0, 180);
    p = end;
    if (*p == ',') p++;
  }
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(angles[i]);
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line[len] = '\0';
      if (len > 0) handleLine(line);
      len = 0;
    } else if (c != '\r') {
      if (len < sizeof(line) - 1) {
        line[len++] = c;
      } else {
        len = 0;  // overflow: drop the line
      }
    }
  }
}
