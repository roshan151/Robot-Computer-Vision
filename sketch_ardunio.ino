/**
 * Drivetrain controller for Arduino — v3
 *
 * Changes vs v2:
 *  1. Implements the M:<dir>,<speed>,<ticks> tick-counted move protocol the
 *     Pi has been sending all along. The Arduino now drives until the average
 *     of |dL| and |dR| reaches the target, then auto-stops and ACKs.
 *  2. Replaces String g_lineBuf with a fixed char buffer. The Arduino UNO's
 *     heap fragments badly under repeated String += operations, which was
 *     the most likely cause of the mid-run "no ACK" hangs.
 *  3. Wires up the host watchdog (it was defined but unused).
 *  4. Adds a per-move safety timeout so a wedged or stalled wheel can't
 *     block the Pi forever.
 *  5. Resets the sync PID accumulator at the start of every M: move.
 */

// --- Pin map ---
static const uint8_t PIN_MOTOR_L_ENA = 5;
static const uint8_t PIN_MOTOR_L_IN1 = 6;
static const uint8_t PIN_MOTOR_L_IN2 = 7;
static const uint8_t PIN_MOTOR_R_ENA = 9;
static const uint8_t PIN_MOTOR_R_IN1 = 10;
static const uint8_t PIN_MOTOR_R_IN2 = 11;

static const uint8_t PIN_ENC_L_A = 2;
static const uint8_t PIN_ENC_L_B = 4;
static const uint8_t PIN_ENC_R_A = 3;
static const uint8_t PIN_ENC_R_B = 8;

// --- Timing ---
static const unsigned long CONTROL_PERIOD_US = 4000;
static const unsigned long WATCHDOG_MS       = 500;     // open-loop only
static const unsigned long ENC_REPORT_MS     = 100;
static const unsigned long MOVE_TIMEOUT_MS   = 10000;   // per M: move
static const uint8_t       PWM_MAX           = 255;

// --- Acceleration ---
static const float ACCEL_STEP = 12.0f;

// --- PID ---
static const float SYNC_KP    = 0.35f;
static const float SYNC_KI    = 0.02f;
static const float SYNC_KD    = 0.08f;
static const float SYNC_I_LIM = 80.0f;
static const float DERR_CLAMP = 1000.0f;

enum DriveMode : uint8_t { MODE_STOP = 0, MODE_FWD, MODE_BACK, MODE_LEFT, MODE_RIGHT };

// --- Encoders ---
volatile long g_encLeft  = 0;
volatile long g_encRight = 0;
static uint8_t g_encLastL = 0;
static uint8_t g_encLastR = 0;

// --- Quadrature ---
static int8_t quadratureDelta(uint8_t prev, uint8_t curr) {
  static const int8_t tbl[16] = {
    0, 1, -1, 0, -1, 0, 0, 1,
    1, 0, 0, -1, 0, -1, 1, 0
  };
  return tbl[(prev << 2) | (curr & 3)];
}

void isrEncLeft() {
  uint8_t a = digitalRead(PIN_ENC_L_A);
  uint8_t b = digitalRead(PIN_ENC_L_B);
  uint8_t curr = a | (b << 1);
  g_encLeft += quadratureDelta(g_encLastL, curr);
  g_encLastL = curr;
}

void isrEncRight() {
  uint8_t a = digitalRead(PIN_ENC_R_A);
  uint8_t b = digitalRead(PIN_ENC_R_B);
  uint8_t curr = a | (b << 1);
  g_encRight += quadratureDelta(g_encLastR, curr);
  g_encLastR = curr;
}

inline long readEncLeft() {
  noInterrupts();
  long c = g_encLeft;
  interrupts();
  return c;
}

inline long readEncRight() {
  noInterrupts();
  long c = g_encRight;
  interrupts();
  return c;
}

// --- Motor control ---
void motorWrite(uint8_t ena, uint8_t in1, uint8_t in2, int16_t pwm) {
  if (pwm == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(ena, 0);
    return;
  }
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(ena, min(pwm, (int16_t)PWM_MAX));
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(ena, min(-pwm, (int16_t)PWM_MAX));
  }
}

// --- State ---
static const uint8_t LINE_BUF_MAX = 40;
static char    g_lineBuf[LINE_BUF_MAX];
static uint8_t g_lineLen = 0;
static bool    g_lineOverflow = false;

unsigned long g_lastHostMs       = 0;
unsigned long g_lastEncReportMs  = 0;
unsigned long g_lastControlUs    = 0;

DriveMode g_mode  = MODE_STOP;
uint8_t   g_speed = 180;

float g_pwmL = 0.0f;
float g_pwmR = 0.0f;

float g_syncI       = 0.0f;
long  g_syncLastErr = 0;
unsigned long g_syncLastUs = 0;

// --- Tick-counted move state ---
bool          g_moveActive  = false;
long          g_moveStartL  = 0;
long          g_moveStartR  = 0;
unsigned long g_moveTicks   = 0;
unsigned long g_moveStartMs = 0;

// --- Helpers ---
float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void emergencyStop() {
  g_mode = MODE_STOP;
  g_pwmL = g_pwmR = 0;
  g_syncI = 0;
  g_syncLastErr = 0;
  motorWrite(PIN_MOTOR_L_ENA, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, 0);
  motorWrite(PIN_MOTOR_R_ENA, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, 0);
}

void cancelMove() {
  g_moveActive = false;
  g_moveTicks  = 0;
}

void sendAck() { Serial.println(F("ACK")); }
void sendErr() { Serial.println(F("ERR")); }

void sendEnc() {
  Serial.print(F("ENC:"));
  Serial.print(readEncLeft());
  Serial.print(F(","));
  Serial.println(readEncRight());
}

void resetWatchdog() {
  g_lastHostMs = millis();
}

// --- Control loop ---
void controlUpdate() {
  int16_t targetL = 0, targetR = 0;
  int mag = g_speed;

  switch (g_mode) {
    case MODE_FWD:   targetL = mag;  targetR = mag;  break;
    case MODE_BACK:  targetL = -mag; targetR = -mag; break;
    case MODE_LEFT:  targetL = -mag; targetR = mag;  g_syncI = 0; break;
    case MODE_RIGHT: targetL = mag;  targetR = -mag; g_syncI = 0; break;
    default: break;
  }

  // PID sync (forward/back only)
  if (g_mode == MODE_FWD || g_mode == MODE_BACK) {
    long err = readEncLeft() - readEncRight();

    unsigned long now = micros();
    float dt = (now - g_syncLastUs) / 1000000.0f;
    if (dt <= 0 || dt > 0.2f) dt = 0.01f;
    g_syncLastUs = now;

    float dErr = (err - g_syncLastErr) / dt;
    dErr = clampf(dErr, -DERR_CLAMP, DERR_CLAMP);

    g_syncLastErr = err;
    g_syncI = clampf(g_syncI + err * dt, -SYNC_I_LIM, SYNC_I_LIM);

    float fix = SYNC_KP * err + SYNC_KI * g_syncI + SYNC_KD * dErr;

    targetL = clampf(targetL - fix, -PWM_MAX, PWM_MAX);
    targetR = clampf(targetR + fix, -PWM_MAX, PWM_MAX);
  }

  // Ramp
  auto ramp = [](float* cur, int16_t goal) {
    if (*cur < goal) *cur = min(*cur + ACCEL_STEP, (float)goal);
    else if (*cur > goal) *cur = max(*cur - ACCEL_STEP, (float)goal);
  };

  ramp(&g_pwmL, targetL);
  ramp(&g_pwmR, targetR);

  motorWrite(PIN_MOTOR_L_ENA, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, g_pwmL);
  motorWrite(PIN_MOTOR_R_ENA, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, g_pwmR);
}

// --- M: parser ---
// Expects "<dir>,<speed>,<ticks>" with dir in {F,B,L,R}, speed 0-255, ticks > 0.
// Returns true on success and starts the move; false if malformed.
bool startTickMove(const char* args) {
  if (args == nullptr || args[0] == '\0' || args[1] != ',') return false;

  char dir = args[0];
  if (dir != 'F' && dir != 'B' && dir != 'L' && dir != 'R') return false;

  // Walk past "<dir>,"
  const char* speedStr = args + 2;
  const char* comma    = strchr(speedStr, ',');
  if (comma == nullptr || comma == speedStr) return false;

  // atoi will stop at the comma
  int  speed = atoi(speedStr);
  long ticks = atol(comma + 1);
  if (speed < 0 || speed > 255) return false;
  if (ticks <= 0) return false;

  g_speed = (uint8_t)speed;
  switch (dir) {
    case 'F': g_mode = MODE_FWD;   break;
    case 'B': g_mode = MODE_BACK;  break;
    case 'L': g_mode = MODE_LEFT;  break;
    case 'R': g_mode = MODE_RIGHT; break;
  }

  // Fresh PID state for this move
  g_syncI       = 0.0f;
  g_syncLastErr = 0;
  g_syncLastUs  = micros();

  g_moveStartL  = readEncLeft();
  g_moveStartR  = readEncRight();
  g_moveTicks   = (unsigned long)ticks;
  g_moveStartMs = millis();
  g_moveActive  = true;
  return true;
}

void handleCommand(char* cmd) {
  // Strip trailing CR/whitespace in place
  while (g_lineLen > 0) {
    char c = cmd[g_lineLen - 1];
    if (c == ' ' || c == '\t' || c == '\r') {
      cmd[--g_lineLen] = '\0';
    } else {
      break;
    }
  }
  if (g_lineLen == 0) return;

  resetWatchdog();

  // Tick-counted move: ACK is sent later, when ticks complete.
  if (cmd[0] == 'M' && cmd[1] == ':') {
    if (!startTickMove(cmd + 2)) {
      sendErr();
    }
    return;
  }

  // Open-loop intent commands: ACK immediately.
  if (g_lineLen == 1) {
    switch (cmd[0]) {
      case 'F': cancelMove(); g_mode = MODE_FWD;   sendAck(); return;
      case 'B': cancelMove(); g_mode = MODE_BACK;  sendAck(); return;
      case 'L': cancelMove(); g_mode = MODE_LEFT;  sendAck(); return;
      case 'R': cancelMove(); g_mode = MODE_RIGHT; sendAck(); return;
      case 'S': cancelMove(); emergencyStop();     sendAck(); return;
    }
  }

  // V:<0-255>
  if (cmd[0] == 'V' && cmd[1] == ':') {
    int val = atoi(cmd + 2);
    g_speed = (uint8_t)constrain(val, 0, 255);
    sendAck();
    return;
  }

  sendErr();
}

// --- Setup ---
void setup() {
  Serial.begin(115200);

  pinMode(PIN_MOTOR_L_ENA, OUTPUT);
  pinMode(PIN_MOTOR_R_ENA, OUTPUT);
  pinMode(PIN_MOTOR_L_IN1, OUTPUT);
  pinMode(PIN_MOTOR_L_IN2, OUTPUT);
  pinMode(PIN_MOTOR_R_IN1, OUTPUT);
  pinMode(PIN_MOTOR_R_IN2, OUTPUT);

  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  g_encLastL = digitalRead(PIN_ENC_L_A) | (digitalRead(PIN_ENC_L_B) << 1);
  g_encLastR = digitalRead(PIN_ENC_R_A) | (digitalRead(PIN_ENC_R_B) << 1);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncRight, CHANGE);

  emergencyStop();
  resetWatchdog();
  g_lastControlUs = micros();
  g_syncLastUs    = g_lastControlUs;

  sendAck();
}

// --- Loop ---
void loop() {
  // ---- Serial ingest (fixed-size buffer, no String) ----
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (g_lineOverflow) {
        // We lost characters; reject this line.
        g_lineLen = 0;
        g_lineOverflow = false;
        sendErr();
      } else if (g_lineLen > 0) {
        g_lineBuf[g_lineLen] = '\0';
        handleCommand(g_lineBuf);
        g_lineLen = 0;
      }
    } else {
      if (g_lineLen < LINE_BUF_MAX - 1) {
        g_lineBuf[g_lineLen++] = c;
      } else {
        g_lineOverflow = true;
      }
    }
  }

  // ---- Control loop ----
  if (micros() - g_lastControlUs >= CONTROL_PERIOD_US) {
    g_lastControlUs = micros();
    controlUpdate();
  }

  // ---- Encoder telemetry ----
  if (millis() - g_lastEncReportMs >= ENC_REPORT_MS) {
    g_lastEncReportMs = millis();
    sendEnc();
  }

  // ---- Tick-counted move completion ----
  if (g_moveActive) {
    long dl = labs(readEncLeft()  - g_moveStartL);
    long dr = labs(readEncRight() - g_moveStartR);
    unsigned long avg = (unsigned long)((dl + dr) / 2);

    if (avg >= g_moveTicks) {
      emergencyStop();
      g_moveActive = false;
      sendAck();
    } else if (millis() - g_moveStartMs > MOVE_TIMEOUT_MS) {
      // Stall / wedge safety: stop and report ERR so the Pi can react.
      emergencyStop();
      g_moveActive = false;
      sendErr();
    }
  }

  // ---- Host watchdog (open-loop only) ----
  // Tick moves are self-terminating, so we exempt them.
  if (!g_moveActive && g_mode != MODE_STOP &&
      (millis() - g_lastHostMs) > WATCHDOG_MS) {
    emergencyStop();
  }
}
