/*
 * Drivetrain Controller — DRV8833 edition
 *
 * Motor driver: DRV8833 (MOSFET H-bridge)
 * Pin mapping (Arduino → DRV8833 module):
 *
 *   Pin 5  (PWM) → IN1   Left  motor forward PWM
 *   Pin 6  (PWM) → IN2   Left  motor reverse PWM
 *   Pin 9  (PWM) → IN3   Right motor forward PWM
 *   Pin 10 (PWM) → IN4   Right motor reverse PWM
 *   5V           → ULT   nSLEEP — must be HIGH to enable chip
 *   GND          → GND
 *   (EEP/nFAULT  → optional Arduino input for fault detection)
 *
 * DRV8833 control truth table (per channel):
 *   IN1=PWM  IN2=LOW  → forward at PWM duty
 *   IN1=LOW  IN2=PWM  → reverse at PWM duty
 *   IN1=LOW  IN2=LOW  → coast (outputs floating)
 *   IN1=HIGH IN2=HIGH → brake (outputs shorted to GND)
 */

#include <Arduino.h>

// ---- Timing / ramp constants ----
#define PWM_MAX           255
#define RAMP_STEP         3      // PWM units per ramp tick
#define RAMP_MS           2      // ramp tick interval (ms)
#define KICK_PWM          85     // pre-seed PWM to beat static friction
#define COAST_MS_SAME     100    // dead-time for non-reversing direction change
#define COAST_MS_REVERSE  200    // dead-time for F↔B reversal
                                 // DRV8833 runs cool — 200 ms is enough

// ---- Sync PID ----
// Proportional gain on left-right encoder difference.
// Increase if drift > 5%; decrease if robot oscillates/weaves.
#define SYNC_KP           0.4f

// ---- Motor pins (DRV8833 — no separate ENA pins) ----
#define IN1_L  5    // Left  motor forward  (PWM)
#define IN2_L  6    // Left  motor reverse  (PWM)
#define IN1_R  9    // Right motor forward  (PWM)
#define IN2_R  10   // Right motor reverse  (PWM)

// ---- Encoder pins ----
#define ENC_L  2    // Left  encoder — interrupt pin
#define ENC_R  3    // Right encoder — interrupt pin

// ---- Optional fault pin ----
// Connect DRV8833 EEP (nFAULT) here to detect overcurrent/thermal events.
// Leave as -1 to disable.
#define FAULT_PIN  -1

// ---- Serial buffer ----
#define CMD_BUF_SIZE 64
char    cmd_buf[CMD_BUF_SIZE];
uint8_t cmd_pos = 0;

// ---- Encoder state ----
// Signed: positive = forward/right-turn, negative = reverse/left-turn.
// enc_dir is set by startMove() before encoders are reset so the very
// first ISR pulse counts in the correct direction.
volatile long enc_left  = 0;
volatile long enc_right = 0;
volatile int  enc_dir   = 1;

// ---- ISR debounce (software low-pass) ----
// Real encoder edges on this drivetrain are spaced >>1 ms apart at full speed
// (right wheel measured ~1300 us between edges at 50% PWM).  Electrical noise
// from the DRV8833's fast MOSFET edges induces interrupt-rate bursts spaced
// only a few microseconds apart, which is how a single noisy line produced
// 1.2 billion counts in 50 ms.
//
// Reject any edge that arrives sooner than MIN_PULSE_US after the previous
// one on the same channel.  150 us gives ~8x margin against the fastest real
// pulse while squashing the ns-scale noise bursts.
//
// If you ever spin the wheels much faster (>3 kHz edge rate), drop this.
#define MIN_PULSE_US  150UL
volatile unsigned long last_left_us  = 0;
volatile unsigned long last_right_us = 0;

// ---- Motion state ----
int  target_pwm_left  = 0;
int  target_pwm_right = 0;
int  current_pwm_left  = 0;
int  current_pwm_right = 0;

long          target_ticks    = 0;
bool          move_active     = false;
char          current_dir     = '\0';
unsigned long move_start_ms   = 0;       // millis() when current move began

// Safety timeout — abort move and send ERR if it takes longer than this.
// Prevents motors running forever if both encoders fail simultaneously.
#define MOVE_TIMEOUT_MS  15000UL

// ---- Ramp timer ----
unsigned long last_ramp_ms = 0;

// --------------------------------------------------
// Encoder ISRs
//
// Debounced: ignore any edge that arrives within MIN_PULSE_US of the
// previous edge on the same channel.  This rejects MOSFET-switching
// noise bursts without requiring a hardware RC filter.
// --------------------------------------------------
void isr_left() {
  unsigned long now = micros();
  if (now - last_left_us < MIN_PULSE_US) return;
  last_left_us = now;
  enc_left += enc_dir;
}
void isr_right() {
  unsigned long now = micros();
  if (now - last_right_us < MIN_PULSE_US) return;
  last_right_us = now;
  enc_right += enc_dir;
}

// --------------------------------------------------
// motorWrite — DRV8833 style.
//
// No separate ENA pin. Direction and speed are both
// controlled by the two IN pins per channel:
//   pwm > 0 → forward:  IN1=PWM, IN2=LOW
//   pwm < 0 → reverse:  IN1=LOW, IN2=PWM
//   pwm = 0 → coast:    IN1=LOW, IN2=LOW
//
// For active braking (motors shorted to GND) use brake().
// --------------------------------------------------
void motorWrite(uint8_t in1, uint8_t in2, int pwm) {
  if (pwm == 0) {
    // Coast — let motors spin down freely.
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else if (pwm > 0) {
    // Forward — PWM on IN1, IN2 held low.
    analogWrite(in1, min(pwm, PWM_MAX));
    digitalWrite(in2, LOW);
  } else {
    // Reverse — IN1 held low, PWM on IN2.
    digitalWrite(in1, LOW);
    analogWrite(in2, min(-pwm, PWM_MAX));
  }
}

// --------------------------------------------------
// immediateStop — coast both motors instantly, no ramp.
// Used in startMove() dead-time and emergency stop.
// --------------------------------------------------
void immediateStop() {
  target_pwm_left  = 0;
  target_pwm_right = 0;
  current_pwm_left  = 0;
  current_pwm_right = 0;
  motorWrite(IN1_L, IN2_L, 0);
  motorWrite(IN1_R, IN2_R, 0);
}

// --------------------------------------------------
// brakeMotors — active brake (both IN pins HIGH).
// Stronger stop than coast — use for quick halts.
// --------------------------------------------------
void brakeMotors() {
  target_pwm_left  = 0;
  target_pwm_right = 0;
  current_pwm_left  = 0;
  current_pwm_right = 0;
  // DRV8833 brake mode: both INx HIGH shorts motor to GND.
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN2_L, HIGH);
  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, HIGH);
}

// --------------------------------------------------
// stopMotors — immediate stop.
//
// Calls immediateStop() to zero PWM outputs instantly
// rather than ramping down.  This ensures motors stop
// the moment the Pi calls close() and sends "S" — the
// ramp-down approach left motors spinning for ~100 ms
// after the port closed, which looked like no-stop.
//
// Used by: S command, move completion, fault handler.
// --------------------------------------------------
void stopMotors() {
  move_active = false;
  immediateStop();
}

// --------------------------------------------------
// PWM ramp helper
// --------------------------------------------------
int rampPWM(int current, int target) {
  if (current < target) {
    current += RAMP_STEP;
    if (current > target) current = target;
  } else if (current > target) {
    current -= RAMP_STEP;
    if (current < target) current = target;
  }
  return current;
}

// --------------------------------------------------
// isReversal — true only for F↔B switches.
// Turns are not reversals — each wheel is already
// opposite so no extra coast is needed.
// --------------------------------------------------
bool isReversal(char prev, char next) {
  return (prev == 'F' && next == 'B') ||
         (prev == 'B' && next == 'F');
}

// --------------------------------------------------
// startMove — begin an encoder-counted move.
//
// Sequence:
//  1. Validate direction (ERR before touching state).
//  2. Direction-aware coast:
//       F↔B reversal → COAST_MS_REVERSE (200 ms)
//       anything else → COAST_MS_SAME   (100 ms)
//  3. Set enc_dir, reset encoder counts.
//  4. Compute PWM signs for each motor.
//  5. Pre-seed current_pwm to KICK_PWM (above static
//     friction threshold) and apply immediately.
//  6. Arm move_active — loop() sends ACK on completion.
// --------------------------------------------------
void startMove(char dir, int speed, long ticks) {
  // 1. Validate.
  if (dir != 'F' && dir != 'B' && dir != 'L' && dir != 'R') {
    Serial.println("ERR");
    return;
  }

  // 2. Coast to stop with direction-aware timing.
  if (move_active || current_pwm_left != 0 || current_pwm_right != 0) {
    immediateStop();
    int coast = isReversal(current_dir, dir) ? COAST_MS_REVERSE : COAST_MS_SAME;
    delay(coast);
  }

  current_dir = dir;

  // 3. Set encoder direction, then atomically reset counts.
  // Also reset ISR debounce timestamps so the first real edge of the new
  // move is not rejected because it happens to fall within MIN_PULSE_US
  // of a stale timestamp from the previous move.
  int new_enc_dir = (dir == 'F' || dir == 'R') ? 1 : -1;
  noInterrupts();
  enc_dir       = new_enc_dir;
  enc_left      = 0;
  enc_right     = 0;
  last_left_us  = 0;
  last_right_us = 0;
  interrupts();

  target_ticks  = ticks;
  move_active   = true;
  move_start_ms = millis();

  // 4. PWM signs per motor per direction.
  int left_sign  = 0;
  int right_sign = 0;
  switch (dir) {
    case 'F': left_sign =  1; right_sign =  1; break;
    case 'B': left_sign = -1; right_sign = -1; break;
    case 'L': left_sign = -1; right_sign =  1; break;
    case 'R': left_sign =  1; right_sign = -1; break;
  }

  target_pwm_left  = left_sign  * speed;
  target_pwm_right = right_sign * speed;

  // 5. Kick-start above static friction, apply immediately.
  int kick = min(speed, KICK_PWM);
  current_pwm_left  = left_sign  * kick;
  current_pwm_right = right_sign * kick;

  motorWrite(IN1_L, IN2_L, current_pwm_left);
  motorWrite(IN1_R, IN2_R, current_pwm_right);

  last_ramp_ms = millis();
  // 6. move_active is true — ACK sent by loop() on completion.
}

// --------------------------------------------------
// handleCommand
// --------------------------------------------------
void handleCommand(char* cmd) {
  if (cmd[0] == '\0') return;

  // STOP — graceful ramp-down.
  if (strcmp(cmd, "S") == 0) {
    stopMotors();
    Serial.println("ACK");
    return;
  }

  // Open-loop intent commands (brain_loop streaming).
  // ACK immediately; no encoder counting.
  if (strcmp(cmd, "F") == 0) {
    move_active = false;
    target_pwm_left  =  150;
    target_pwm_right =  150;
    Serial.println("ACK");
    return;
  }
  if (strcmp(cmd, "B") == 0) {
    move_active = false;
    target_pwm_left  = -150;
    target_pwm_right = -150;
    Serial.println("ACK");
    return;
  }
  if (strcmp(cmd, "L") == 0) {
    move_active = false;
    target_pwm_left  = -150;
    target_pwm_right =  150;
    Serial.println("ACK");
    return;
  }
  if (strcmp(cmd, "R") == 0) {
    move_active = false;
    target_pwm_left  =  150;
    target_pwm_right = -150;
    Serial.println("ACK");
    return;
  }

  // Encoder-counted move: M:<dir>,<speed>,<ticks>
  if (strncmp(cmd, "M:", 2) == 0) {
    char dir;
    int  speed;
    long ticks;
    int parsed = sscanf(cmd, "M:%c,%d,%ld", &dir, &speed, &ticks);
    if (parsed != 3 || speed <= 0 || ticks <= 0) {
      Serial.println("ERR");
      return;
    }
    startMove(dir, speed, ticks);
    return;  // ACK sent by loop() on completion
  }

  // Speed override: V:<pwm>  (0-255, open-loop only)
  if (strncmp(cmd, "V:", 2) == 0) {
    int v = 0;
    if (sscanf(cmd, "V:%d", &v) == 1) {
      v = constrain(v, 0, 255);
      if (target_pwm_left  > 0) target_pwm_left  =  v;
      if (target_pwm_left  < 0) target_pwm_left  = -v;
      if (target_pwm_right > 0) target_pwm_right =  v;
      if (target_pwm_right < 0) target_pwm_right = -v;
      Serial.println("ACK");
    } else {
      Serial.println("ERR");
    }
    return;
  }

  Serial.println("ERR");
}

// --------------------------------------------------
// Setup
// --------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Motor output pins.
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  // Encoder input pins — internal pull-up for open-collector encoders.
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  // Optional fault pin.
  if (FAULT_PIN >= 0) {
    pinMode(FAULT_PIN, INPUT_PULLUP);  // nFAULT is open-drain, active LOW
  }

  attachInterrupt(digitalPinToInterrupt(ENC_L), isr_left,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), isr_right, RISING);

  immediateStop();
  delay(200);
  Serial.println("BOOT");
}

// --------------------------------------------------
// Loop
// --------------------------------------------------
void loop() {

  // ---- Serial command reader ----
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd_pos > 0) {
        cmd_buf[cmd_pos] = '\0';
        handleCommand(cmd_buf);
        cmd_pos = 0;
      }
    } else {
      if (cmd_pos < CMD_BUF_SIZE - 1) {
        cmd_buf[cmd_pos++] = c;
      } else {
        cmd_pos = 0;  // buffer overflow — discard and resync
      }
    }
  }

  // ---- Optional fault detection ----
  // DRV8833 pulls nFAULT LOW on overcurrent or thermal event.
  // We coast-stop and report ERR so the Pi can detect it.
  if (FAULT_PIN >= 0 && digitalRead(FAULT_PIN) == LOW) {
    if (move_active) {
      immediateStop();
      move_active = false;
      Serial.println("ERR");
    }
  }

  // ---- Timed PWM ramp + sync PID ----
  unsigned long now = millis();
  if (now - last_ramp_ms >= RAMP_MS) {
    last_ramp_ms = now;

    if (move_active) {
      // Sync PID — both encoders are signed and count in the same
      // direction, so diff > 0 always means left is ahead of right.
      noInterrupts();
      long el = enc_left;
      long er = enc_right;
      interrupts();

      long diff = el - er;

      // Clamp diff before feeding the PID.  A real left-right mismatch is
      // tens of ticks at most; anything larger is almost certainly a noise
      // glitch on one channel.  Clamping prevents (a) int overflow when
      // multiplying by SYNC_KP and casting to int, and (b) a single bad
      // sample slamming PWM to zero on the "ahead" wheel.
      if (diff >  500) diff =  500;
      if (diff < -500) diff = -500;

      if (target_pwm_left != 0 && target_pwm_right != 0) {
        int correction = (int)(SYNC_KP * (float)diff);
        int tl = target_pwm_left  - correction;
        int tr = target_pwm_right + correction;

        // Clamp — never let PID flip a wheel's direction mid-move.
        if (target_pwm_left  > 0) tl = max(tl, 0);
        if (target_pwm_left  < 0) tl = min(tl, 0);
        if (target_pwm_right > 0) tr = max(tr, 0);
        if (target_pwm_right < 0) tr = min(tr, 0);

        current_pwm_left  = rampPWM(current_pwm_left,  tl);
        current_pwm_right = rampPWM(current_pwm_right, tr);
      } else {
        current_pwm_left  = rampPWM(current_pwm_left,  target_pwm_left);
        current_pwm_right = rampPWM(current_pwm_right, target_pwm_right);
      }
    } else {
      current_pwm_left  = rampPWM(current_pwm_left,  target_pwm_left);
      current_pwm_right = rampPWM(current_pwm_right, target_pwm_right);
    }

    motorWrite(IN1_L, IN2_L, current_pwm_left);
    motorWrite(IN1_R, IN2_R, current_pwm_right);
  }

  // ---- Move completion + safety timeout + noise sanity ----
  //
  // Completion uses min() of the two wheels rather than max().
  // Rationale: a noisy encoder line (capacitive coupling from MOSFET
  // switching) can fire interrupts at the AVR's service rate, producing
  // 10^9-scale counts in milliseconds.  max() would treat that as "done"
  // and end the move with the robot having barely moved.  min() requires
  // BOTH wheels to reach the target, so a single noisy wheel can no
  // longer end a move prematurely.  The MOVE_TIMEOUT_MS safety net still
  // protects against the case where one wheel genuinely stalls.
  //
  // Noise sanity: if the two wheels diverge wildly mid-move, one channel
  // is almost certainly counting noise.  Stop and report ERR so the Pi
  // can react instead of letting the move drift to the timeout.
  if (move_active) {
    noInterrupts();
    long el = enc_left;
    long er = enc_right;
    interrupts();

    long abs_el = abs(el);
    long abs_er = abs(er);
    long travelled = min(abs_el, abs_er);

    // Detect runaway noise: one wheel reports >>10x the other once both
    // have moved enough that the ratio is meaningful (>50 ticks).  This
    // catches the "left encoder spewing 10^9 counts" failure mode early.
    bool noise_runaway = false;
    if (abs_el > 50 && abs_er > 50) {
      if (abs_el > 10 * abs_er || abs_er > 10 * abs_el) {
        noise_runaway = true;
      }
    } else if (abs_el > 10000 && abs_er < 10) {
      noise_runaway = true;
    } else if (abs_er > 10000 && abs_el < 10) {
      noise_runaway = true;
    }

    if (travelled >= target_ticks) {
      stopMotors();
      Serial.println("ACK");
    } else if (noise_runaway) {
      stopMotors();
      Serial.println("ERR");   // noisy encoder line — Pi should abort
    } else if (millis() - move_start_ms > MOVE_TIMEOUT_MS) {
      stopMotors();
      Serial.println("ERR");   // Pi sees ERR and aborts cleanly
    }
  }

  // ---- Encoder telemetry (every 100 ms) ----
  static unsigned long last_enc_print = 0;
  if (millis() - last_enc_print >= 100) {
    last_enc_print = millis();
    noInterrupts();
    long el = enc_left;
    long er = enc_right;
    interrupts();
    Serial.print("ENC:");
    Serial.print(el);
    Serial.print(",");
    Serial.println(er);
  }
}
