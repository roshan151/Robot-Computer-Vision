/**
 * Drive debug sketch — DRV8833 edition
 *
 * Prints every decision the firmware makes so you can see exactly
 * what is happening at each stage of a move.
 *
 * Upload this, open Serial Monitor at 115200 baud with "Newline" line
 * endings, then send:  M:F,180,400
 *
 * Output you will see:
 *   DBG:CMD>...            — raw command received
 *   DBG:MOVE_START         — M: parsed OK, move begun
 *   DBG:ENC L=n R=n        — tick counts printed every 200 ms while moving
 *   ACK                    — move completed (tick target reached)
 *   ERR                    — move timed out (encoders not counting)
 *   DBG:PARSE_FAIL         — M: format was wrong
 *   DBG:FAULT              — DRV8833 nFAULT triggered (if EEP pin wired)
 *
 * DRV8833 pin wiring:
 *   Arduino Pin 5  (PWM) → IN1   Left  motor forward
 *   Arduino Pin 6  (PWM) → IN2   Left  motor reverse
 *   Arduino Pin 9  (PWM) → IN3   Right motor forward
 *   Arduino Pin 10 (PWM) → IN4   Right motor reverse
 *   Arduino 5V           → ULT   (nSLEEP — must be HIGH)
 *   Arduino GND          → GND
 *   Buck converter 10V   → VCC
 *
 * DRV8833 control truth table per channel:
 *   IN1=PWM  IN2=LOW  → forward
 *   IN1=LOW  IN2=PWM  → reverse
 *   IN1=LOW  IN2=LOW  → coast
 *   IN1=HIGH IN2=HIGH → brake
 */

// ── Pin map ───────────────────────────────────────────────────────────────
// DRV8833 — no separate ENA pins. IN1/IN2 carry both direction and PWM.
const uint8_t PIN_IN1_L = 5;    // Left  motor forward  (PWM)
const uint8_t PIN_IN2_L = 6;    // Left  motor reverse  (PWM)
const uint8_t PIN_IN1_R = 9;    // Right motor forward  (PWM)
const uint8_t PIN_IN2_R = 10;   // Right motor reverse  (PWM)

const uint8_t PIN_ENC_L = 2;    // Left  encoder — interrupt-capable (INT0)
const uint8_t PIN_ENC_R = 3;    // Right encoder — interrupt-capable (INT1)

// Set to the Arduino pin connected to DRV8833 EEP (nFAULT) if wired.
// nFAULT is open-drain active-LOW — pulled HIGH internally.
// Set to 255 to disable fault detection.
const uint8_t PIN_FAULT = 255;

// ── Timing ────────────────────────────────────────────────────────────────
const unsigned long MOVE_TIMEOUT_MS = 10000;  // abort if move takes longer
const unsigned long DEBUG_PRINT_MS  = 200;    // encoder report interval
const uint8_t       PWM_MAX         = 255;

// ── Encoders ──────────────────────────────────────────────────────────────
volatile long g_encL = 0;
volatile long g_encR = 0;

void isrL() { g_encL++; }
void isrR() { g_encR++; }

long readL() { noInterrupts(); long v = g_encL; interrupts(); return v; }
long readR() { noInterrupts(); long v = g_encR; interrupts(); return v; }

// ── Motor helpers — DRV8833 style ─────────────────────────────────────────
// pwm > 0 → forward:  IN1=PWM, IN2=LOW
// pwm < 0 → reverse:  IN1=LOW, IN2=PWM (magnitude)
// pwm = 0 → coast:    both LOW
void motorWrite(uint8_t in1, uint8_t in2, int16_t pwm) {
  if (pwm == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else if (pwm > 0) {
    analogWrite(in1, min((int16_t)PWM_MAX, pwm));
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    analogWrite(in2, min((int16_t)PWM_MAX, (int16_t)-pwm));
  }
}

// Coast both motors immediately (no ramp).
void stopAll() {
  motorWrite(PIN_IN1_L, PIN_IN2_L, 0);
  motorWrite(PIN_IN1_R, PIN_IN2_R, 0);
}

// ── Serial line buffer ────────────────────────────────────────────────────
const uint8_t LINE_BUF = 48;
char    g_buf[LINE_BUF];
uint8_t g_len = 0;

// ── Move state ────────────────────────────────────────────────────────────
bool          g_active      = false;
long          g_startL      = 0;
long          g_startR      = 0;
unsigned long g_targetTicks = 0;
unsigned long g_startMs     = 0;
unsigned long g_lastDbgMs   = 0;
int16_t       g_pwm         = 0;
int8_t        g_signL       = 1;
int8_t        g_signR       = 1;

// ── Command handler ───────────────────────────────────────────────────────
void handleLine(char* cmd, uint8_t len) {
  // Strip trailing whitespace / CR.
  while (len > 0 && (cmd[len-1] == '\r' || cmd[len-1] == ' ')) cmd[--len] = '\0';
  if (len == 0) return;

  Serial.print(F("DBG:CMD>")); Serial.println(cmd);

  // S = stop
  if (len == 1 && cmd[0] == 'S') {
    g_active = false;
    stopAll();
    Serial.println(F("ACK"));
    return;
  }

  // M:<dir>,<pwm>,<ticks>
  if (len >= 4 && cmd[0] == 'M' && cmd[1] == ':') {
    char  dir      = cmd[2];
    char* speedStr = cmd + 4;           // past "M:X,"
    char* comma    = strchr(speedStr, ',');

    if (cmd[3] != ',' || comma == nullptr ||
        (dir != 'F' && dir != 'B' && dir != 'L' && dir != 'R')) {
      Serial.println(F("DBG:PARSE_FAIL — bad format, expected M:F,180,400"));
      Serial.println(F("ERR"));
      return;
    }

    int  spd   = atoi(speedStr);
    long ticks = atol(comma + 1);

    if (spd < 1 || spd > 255 || ticks < 1) {
      Serial.print(F("DBG:PARSE_FAIL — spd=")); Serial.print(spd);
      Serial.print(F(" ticks="));               Serial.println(ticks);
      Serial.println(F("ERR"));
      return;
    }

    // Set direction signs.
    switch (dir) {
      case 'F': g_signL =  1; g_signR =  1; break;
      case 'B': g_signL = -1; g_signR = -1; break;
      case 'L': g_signL = -1; g_signR =  1; break;
      case 'R': g_signL =  1; g_signR = -1; break;
    }

    g_pwm         = (int16_t)spd;
    g_startL      = readL();
    g_startR      = readR();
    g_targetTicks = (unsigned long)ticks;
    g_startMs     = millis();
    g_lastDbgMs   = g_startMs;
    g_active      = true;

    // Apply motor power immediately.
    motorWrite(PIN_IN1_L, PIN_IN2_L, g_signL * g_pwm);
    motorWrite(PIN_IN1_R, PIN_IN2_R, g_signR * g_pwm);

    Serial.print(F("DBG:MOVE_START dir=")); Serial.print(dir);
    Serial.print(F(" pwm="));               Serial.print(g_pwm);
    Serial.print(F(" target="));            Serial.print(g_targetTicks);
    Serial.print(F(" encL0="));             Serial.print(g_startL);
    Serial.print(F(" encR0="));             Serial.println(g_startR);
    return;
  }

  Serial.println(F("DBG:UNKNOWN_CMD"));
  Serial.println(F("ERR"));
}

// ── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1_L, OUTPUT);
  pinMode(PIN_IN2_L, OUTPUT);
  pinMode(PIN_IN1_R, OUTPUT);
  pinMode(PIN_IN2_R, OUTPUT);

  pinMode(PIN_ENC_L, INPUT_PULLUP);
  pinMode(PIN_ENC_R, INPUT_PULLUP);

  if (PIN_FAULT != 255) {
    pinMode(PIN_FAULT, INPUT_PULLUP);  // nFAULT is open-drain active-LOW
  }

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), isrR, RISING);

  stopAll();
  Serial.println(F("ACK"));
  Serial.println(F("DBG:DRV8833 ready. Send M:F,180,400 to test (115200 baud, Newline ending)"));
  Serial.println(F("DBG:Pins — IN1_L=5 IN2_L=6 IN1_R=9 IN2_R=10 ENC_L=2 ENC_R=3"));
}

// ── Loop ──────────────────────────────────────────────────────────────────
void loop() {

  // ---- Serial command reader ----
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_len > 0) {
        g_buf[g_len] = '\0';
        handleLine(g_buf, g_len);
        g_len = 0;
      }
    } else if (g_len < LINE_BUF - 1) {
      g_buf[g_len++] = c;
    }
  }

  // ---- DRV8833 fault detection ----
  // nFAULT (EEP) goes LOW on overcurrent or thermal shutdown.
  // DRV8833 auto-recovers — we just coast-stop and report.
  if (PIN_FAULT != 255 && digitalRead(PIN_FAULT) == LOW) {
    if (g_active) {
      stopAll();
      g_active = false;
      Serial.println(F("DBG:FAULT — DRV8833 nFAULT triggered (overcurrent or thermal)"));
      Serial.println(F("ERR"));
    }
  }

  // ---- Active move monitoring ----
  if (g_active) {
    long dL  = labs(readL() - g_startL);
    long dR  = labs(readR() - g_startR);
    unsigned long avg = (unsigned long)((dL + dR) / 2);
    unsigned long now = millis();

    // Periodic encoder report so you can watch ticks climbing.
    if (now - g_lastDbgMs >= DEBUG_PRINT_MS) {
      g_lastDbgMs = now;
      Serial.print(F("DBG:ENC L=")); Serial.print(dL);
      Serial.print(F(" R="));        Serial.print(dR);
      Serial.print(F(" avg="));      Serial.print(avg);
      Serial.print(F(" target="));   Serial.println(g_targetTicks);
    }

    // Move complete.
    if (avg >= g_targetTicks) {
      stopAll();
      g_active = false;
      Serial.println(F("ACK"));

    // Move timed out — encoders not counting (stall, wiring fault, etc).
    } else if (now - g_startMs > MOVE_TIMEOUT_MS) {
      stopAll();
      g_active = false;
      Serial.print(F("DBG:TIMEOUT dL=")); Serial.print(dL);
      Serial.print(F(" dR="));            Serial.println(dR);
      Serial.println(F("ERR"));
    }
  }
}
