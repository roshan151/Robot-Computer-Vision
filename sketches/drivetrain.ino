/*
 * Drivetrain Controller — DRV8833 edition
 *
 * Motor driver: DRV8833 breakout (12-pin module).
 * Module pinout:
 *   Side 1:  IN1  IN2  VCC  GND  IN3  IN4
 *   Side 2:  EEP  OUT1 OUT2 OUT3 OUT4 ULT
 *
 * Wiring:
 *   Arduino pin 5  (PWM) → IN1   Left  motor forward
 *   Arduino pin 6  (PWM) → IN2   Left  motor reverse
 *   Arduino pin 9  (PWM) → IN3   Right motor forward
 *   Arduino pin 10 (PWM) → IN4   Right motor reverse
 *   Arduino 3.3V         → EEP   (sleep pin — must be HIGH or chip is off)
 *   Star ground          → GND
 *   Buck converter 10V   → VCC   (single supply: powers motors AND chip)
 *   ULT (fault output)   → unconnected
 *
 * DRV8833 control truth table (per channel):
 *   IN1=PWM  IN2=LOW  → forward at PWM duty
 *   IN1=LOW  IN2=PWM  → reverse at PWM duty
 *   IN1=LOW  IN2=LOW  → coast (outputs floating, wheels spin freely)
 *   IN1=HIGH IN2=HIGH → brake (motor shorted through driver — stops fast)
 *
 * Key features:
 *  1. Reset-cause reporting: every boot prints "BOOT:<hex>" where the hex
 *     value says WHY the chip restarted (power loss / brown-out / reset
 *     pin / watchdog).  See decode table in serial_protocol.py.
 *  2. Active-brake dead-time between moves (150 ms same direction,
 *     400 ms for a forward↔reverse flip) so wheels are truly stopped
 *     before current flows the other way.
 *  3. Soft start: PWM always ramps up from zero (RAMP_STEP per RAMP_MS)
 *     — no instantaneous current step at move start.
 *  4. Sync PID — proportional correction keeps wheels in step.
 *  5. Signed encoder counting (enc_dir set per move direction).
 *  6. Moves end with an active brake for accurate stopping distance.
 */

#include <Arduino.h>

// ---- Reset-cause capture ----
// The AVR chip records WHY it last reset in the MCUSR register:
//   bit0 PORF  = power-on   (5V supply dropped completely — e.g. USB blip)
//   bit1 EXTRF = reset pin  (DTR pulse when the port opens, or noise)
//   bit2 BORF  = brown-out  (5V rail sagged below the safe threshold)
//   bit3 WDRF  = watchdog
// Most Arduino bootloaders clear MCUSR before our code runs, but first
// copy its value into CPU register r2.  This .init0 stub runs before
// everything else and saves r2 so setup() can report the true cause.
uint8_t reset_flags __attribute__((section(".noinit")));
void resetFlagsInit(void) __attribute__((naked)) __attribute__((used)) __attribute__((section(".init0")));
void resetFlagsInit(void) {
  __asm__ __volatile__ ("sts %0, r2\n" : "=m" (reset_flags));
}

// ---- Timing / ramp constants ----
#define PWM_MAX           255
#define RAMP_STEP         3      // PWM units per ramp tick
#define RAMP_MS           2      // ramp tick interval (ms)
#define BRAKE_MS_SAME     150    // active-brake dead-time, same direction
#define BRAKE_MS_REVERSE  400    // active-brake dead-time, F↔B reversal

// ---- Sync PID ----
// Proportional gain on left-right encoder difference.
// Increase if drift > 5%; decrease if robot oscillates/weaves.
#define SYNC_KP           0.4f

// ---- Motor pins (DRV8833 — no separate enable pins) ----
#define IN1_L  5    // Left  motor forward  (PWM)
#define IN2_L  6    // Left  motor reverse  (PWM)
#define IN1_R  9    // Right motor forward  (PWM)
#define IN2_R  10   // Right motor reverse  (PWM)

// ---- Encoder pins ----
#define ENC_L  2    // Left  encoder — interrupt pin
#define ENC_R  3    // Right encoder — interrupt pin

// ---- DRV8833 sleep pin ----
// Drive EEP (nSLEEP) from a digital pin instead of the 3.3V rail.
// The 3.3V pin on FTDI-based boards comes from a tiny, fragile
// regulator — wiring it to the motor driver couples switching noise
// straight into the USB-serial chip.  A digital pin is a buffered,
// robust driver.  HIGH = driver awake.
#define SLEEP_PIN  7

// ---- Serial buffer ----
#define CMD_BUF_SIZE 64
char     cmd_buf[CMD_BUF_SIZE];
uint8_t  cmd_pos    = 0;
uint16_t junk_lines = 0;   // unrecognised lines seen (corrupted input)

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

// ---- Pending move (armed while the brake dead-time runs) ----
// The dead-time must NOT be a delay(): blocking the loop leaves the serial
// port unread, and any byte that arrives meanwhile — including a noise
// glitch — sits in the UART buffer and is parsed as a command as soon as
// the delay ends.  Instead we brake, remember what to do, and let loop()
// start the move once the deadline passes.
bool          brake_pending   = false;
unsigned long brake_until_ms  = 0;
char          pending_dir     = '\0';
int           pending_speed   = 0;
long          pending_ticks   = 0;

// Safety timeout — abort move and send ERR if it takes longer than this.
// Prevents motors running forever if both encoders fail simultaneously.
#define MOVE_TIMEOUT_MS  15000UL

// ---- Ramp timer ----
unsigned long last_ramp_ms = 0;

// ---- Noise tolerance ----
// A corrupted encoder reading is repaired rather than fatal; give up only
// if it keeps happening within a single move.
#define MAX_NOISE_CORRECTIONS 5
int noise_corrections = 0;

// Ticks a wheel can physically produce per millisecond, with headroom.
// Measured peak on this drivetrain is ~0.7 ticks/ms at 50% PWM, so ~1.4
// at full speed; 3 leaves ample margin while still rejecting garbage.
#define MAX_TICKS_PER_MS 3L

// Arduino's abs() is a macro that can truncate to 16 bits depending on
// which definition wins — hence a reading printed as exactly 65535
// (0xFFFF).  Use explicit 32-bit arithmetic for encoder counts.
static inline long labs32(long v) { return v < 0 ? -v : v; }

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
// Direction and speed are both controlled by the two
// IN pins per channel:
//   pwm > 0 → forward:  IN1=PWM, IN2=LOW
//   pwm < 0 → reverse:  IN1=LOW, IN2=PWM
//   pwm = 0 → coast:    IN1=LOW, IN2=LOW
//
// For active braking (motor shorted) use brakeMotors().
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
// coastMotors — outputs floating, wheels spin freely.
// Used at boot and to release the brake before a move.
// --------------------------------------------------
void coastMotors() {
  target_pwm_left  = 0;
  target_pwm_right = 0;
  current_pwm_left  = 0;
  current_pwm_right = 0;
  motorWrite(IN1_L, IN2_L, 0);
  motorWrite(IN1_R, IN2_R, 0);
}

// --------------------------------------------------
// brakeMotors — active brake (both IN pins HIGH).
//
// Shorts each motor through the driver so its own spin
// energy burns off in the motor winding — the wheels
// stop fast and, importantly, no surge is pushed back
// into the power supply.  Once the wheel is stopped a
// braked motor draws no current, so it is safe to leave
// the brake engaged indefinitely.
// --------------------------------------------------
void brakeMotors() {
  target_pwm_left  = 0;
  target_pwm_right = 0;
  current_pwm_left  = 0;
  current_pwm_right = 0;
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN2_L, HIGH);
  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, HIGH);
}

// --------------------------------------------------
// stopMotors — end any move with an active brake.
//
// Braking (not coasting) means the robot stops where the
// encoders say it should, instead of rolling on for an
// extra unmeasured distance.
//
// Used by: S command, move completion, move timeout.
// --------------------------------------------------
void stopMotors() {
  move_active = false;
  brakeMotors();
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
// wheelSigns — which way each wheel turns for a command.
// --------------------------------------------------
void wheelSigns(char dir, int &left, int &right) {
  switch (dir) {
    case 'F': left =  1; right =  1; break;
    case 'B': left = -1; right = -1; break;
    case 'L': left = -1; right =  1; break;
    case 'R': left =  1; right = -1; break;
    default:  left =  0; right =  0; break;   // unknown / first move
  }
}

// --------------------------------------------------
// isReversal — true if EITHER wheel has to change its
// direction of rotation.
//
// Checking only F<->B was wrong: going from B to L flips
// the right wheel from backward to forward, and F to R
// flips the right wheel too.  Those transitions need the
// same dead-time as a straight reversal, or the driver
// pushes current into a wheel that is still turning the
// other way.
// --------------------------------------------------
bool isReversal(char prev, char next) {
  int pl, pr, nl, nr;
  wheelSigns(prev, pl, pr);
  wheelSigns(next, nl, nr);
  if (pl == 0 || nl == 0) return false;   // no known previous direction
  return (pl * nl < 0) || (pr * nr < 0);
}

// --------------------------------------------------
// beginDrive — actually start driving.
//
// Called either directly by startMove() (when no brake
// dead-time is needed) or by loop() once the dead-time
// deadline has passed.
// --------------------------------------------------
void beginDrive(char dir, int speed, long ticks) {
  coastMotors();  // release the brake — loop() ramps PWM up from zero

  current_dir = dir;

  // Set encoder direction, then atomically reset counts.
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

  // Target PWM signs per motor per direction.  current_pwm is 0
  // (coastMotors above), so loop() soft-starts both motors.
  int left_sign, right_sign;
  wheelSigns(dir, left_sign, right_sign);
  target_pwm_left  = left_sign  * speed;
  target_pwm_right = right_sign * speed;

  noise_corrections = 0;

  last_ramp_ms = millis();
  // ACK is sent by loop() once the encoder target is reached.
}

// --------------------------------------------------
// startMove — begin an encoder-counted move.
//
// If the wheels may still be turning, engage the active
// brake and arm a NON-BLOCKING dead-time; loop() calls
// beginDrive() when it expires.  Blocking here with
// delay() would leave the serial port unread for the
// whole dead-time.
// --------------------------------------------------
void startMove(char dir, int speed, long ticks) {
  if (dir != 'F' && dir != 'B' && dir != 'L' && dir != 'R') {
    Serial.println("ERR:BADDIR");
    return;
  }

  // A reversal always gets the hold, even if PWM already reads zero —
  // the wheels may still be spinning down from the previous move.
  bool driving = move_active || current_pwm_left != 0 || current_pwm_right != 0;
  bool reversing = isReversal(current_dir, dir);

  if (driving || reversing) {
    brakeMotors();
    brake_until_ms = millis() + (reversing ? BRAKE_MS_REVERSE : BRAKE_MS_SAME);
    pending_dir    = dir;
    pending_speed  = speed;
    pending_ticks  = ticks;
    brake_pending  = true;
    move_active    = false;   // encoder counting starts in beginDrive()
    return;
  }

  beginDrive(dir, speed, ticks);
}

// --------------------------------------------------
// handleCommand
// --------------------------------------------------
void handleCommand(char* cmd) {
  if (cmd[0] == '\0') return;

  // STOP — immediate active brake, and drop any armed move.
  if (strcmp(cmd, "S") == 0) {
    brake_pending = false;
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
      Serial.println("ERR:PARSE");
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
      Serial.println("ERR:PARSE");
    }
    return;
  }

  // Unrecognised line — corrupted input, since the Pi only ever sends
  // commands this firmware knows.  Do NOT answer with ERR: that is what
  // made noise abort healthy moves.  Report it as a rate-limited warning
  // so a flood of junk can never saturate the transmit buffer and stall
  // the loop (which starved real commands of their ACK).
  static unsigned long last_junk_ms = 0;
  junk_lines++;
  if (millis() - last_junk_ms >= 1000) {
    last_junk_ms = millis();
    Serial.print("WARN:JUNK,");
    Serial.print(junk_lines);
    Serial.print(",");
    cmd[16] = '\0';          // bound the echo — never dump a long line
    Serial.println(cmd);
  }
}

// --------------------------------------------------
// Setup
// --------------------------------------------------
void setup() {
  // Grab the reset cause: prefer the raw MCUSR register if the
  // bootloader left it intact, else use the copy stashed from r2.
  uint8_t cause = MCUSR ? MCUSR : reset_flags;
  cause &= 0x0F;   // only the low 4 bits are reset flags
  MCUSR = 0;

  Serial.begin(115200);

  // Motor output pins.
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  // Wake the DRV8833 (EEP/nSLEEP wired to this pin).
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, HIGH);

  // Encoder input pins — internal pull-up for open-collector encoders.
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L), isr_left,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), isr_right, RISING);

  coastMotors();
  delay(200);

  // "BOOT:<hex>" — the hex digit says why we restarted:
  //   1=power-on  2=reset pin  4=brown-out  8=watchdog  (bits may combine)
  Serial.print("BOOT:");
  Serial.println(cause, HEX);
}

// --------------------------------------------------
// Loop
// --------------------------------------------------
void loop() {

  // ---- Serial command reader ----
  // Noise bytes are DROPPED individually rather than poisoning the whole
  // line.  An earlier version flagged the line as dirty and discarded it,
  // which threw away the real command that followed a noise byte and left
  // the Pi waiting for an ACK that never came.  Filtering per byte means
  // "<noise>S\n" still executes as "S".
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd_pos > 0) {
        cmd_buf[cmd_pos] = '\0';
        handleCommand(cmd_buf);
        cmd_pos = 0;
      }
    } else if (c >= 32 && c <= 126) {          // printable — keep
      if (cmd_pos < CMD_BUF_SIZE - 1) {
        cmd_buf[cmd_pos++] = c;
      } else {
        cmd_pos = 0;   // buffer overflow — discard and resync
      }
    }
    // non-printable bytes fall through and are discarded
  }

  // ---- Non-blocking brake dead-time ----
  // Start the armed move once the wheels have had time to stop.  Doing
  // this here rather than with delay() in startMove() keeps the serial
  // port serviced throughout.
  if (brake_pending && (long)(millis() - brake_until_ms) >= 0) {
    brake_pending = false;
    beginDrive(pending_dir, pending_speed, pending_ticks);
  }

  // ---- Timed PWM ramp + sync PID ----
  unsigned long now = millis();
  if (now - last_ramp_ms >= RAMP_MS) {
    last_ramp_ms = now;

    int tl = target_pwm_left;
    int tr = target_pwm_right;

    if (move_active && tl != 0 && tr != 0) {
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

      int correction = (int)(SYNC_KP * (float)diff);
      tl -= correction;
      tr += correction;

      // The correction may only SLOW a wheel — never drive it past the
      // speed the caller asked for.  Previously only the lower bound was
      // clamped, so the lagging wheel was boosted to catch up: on this
      // drivetrain the right wheel freewheels ~1.6x faster than the left,
      // so the left motor was pushed toward full PWM on every encoder-
      // counted move.  That is current the caller never asked for, drawn
      // only in encoder mode — which is why the open-loop timed test runs
      // clean while these moves brown the board out.  Slowing the leading
      // wheel achieves the same sync and can only ever reduce current.
      if      (target_pwm_left  > 0) tl = constrain(tl, 0, target_pwm_left);
      else if (target_pwm_left  < 0) tl = constrain(tl, target_pwm_left, 0);
      if      (target_pwm_right > 0) tr = constrain(tr, 0, target_pwm_right);
      else if (target_pwm_right < 0) tr = constrain(tr, target_pwm_right, 0);
    }

    int new_l = rampPWM(current_pwm_left,  tl);
    int new_r = rampPWM(current_pwm_right, tr);
    bool changed = (new_l != current_pwm_left) || (new_r != current_pwm_right);
    current_pwm_left  = new_l;
    current_pwm_right = new_r;

    // Only touch the pins when there is something to drive or a change
    // to apply.  Writing "0" unconditionally would release the active
    // brake that stopMotors()/brakeMotors() engaged.
    if (changed || current_pwm_left != 0 || current_pwm_right != 0) {
      motorWrite(IN1_L, IN2_L, current_pwm_left);
      motorWrite(IN1_R, IN2_R, current_pwm_right);
    }
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
  // Noise sanity: judge each reading against how far the wheel COULD have
  // turned in the time elapsed, rather than against the other wheel.
  //
  // The previous ratio test compared abs_el against 10 * abs_er.  Once a
  // count grew past ~215 million that multiply overflowed a signed 32-bit
  // long and went negative, so the comparison was always true and the
  // guard fired forever — even with both wheels reading identically.  And
  // the repair used min() of the two channels, which is a no-op when BOTH
  // are corrupt.  Together they turned one glitch into a guaranteed abort.
  //
  // A time-based ceiling cannot overflow, needs no comparison between the
  // wheels, and rejects any impossible value whatever its origin.
  if (move_active) {
    noInterrupts();
    long el = enc_left;
    long er = enc_right;
    interrupts();

    long abs_el = labs32(el);
    long abs_er = labs32(er);

    // Most ticks the wheel could physically have produced by now, with
    // generous headroom (measured peak is well under 1.5 ticks/ms).
    unsigned long move_ms = millis() - move_start_ms;
    long ceiling = (long)(move_ms + 100UL) * MAX_TICKS_PER_MS;

    bool el_bad = abs_el > ceiling;
    bool er_bad = abs_er > ceiling;

    if (el_bad || er_bad) {
      noise_corrections++;
      if (noise_corrections > MAX_NOISE_CORRECTIONS) {
        stopMotors();
        Serial.print("ERR:NOISE,");
        Serial.print(el);
        Serial.print(",");
        Serial.println(er);
      } else {
        // Replace an impossible reading with the other wheel if that one
        // is still credible, otherwise with the ceiling.  Sign comes from
        // enc_dir, since a corrupted counter's own sign is meaningless.
        long fixed_l = el_bad ? (er_bad ? ceiling : abs_er) : abs_el;
        long fixed_r = er_bad ? (el_bad ? ceiling : abs_el) : abs_er;
        noInterrupts();
        enc_left  = (enc_dir < 0) ? -fixed_l : fixed_l;
        enc_right = (enc_dir < 0) ? -fixed_r : fixed_r;
        interrupts();

        // Rate-limited: a burst must not flood the transmit buffer.
        static unsigned long last_warn_ms = 0;
        if (millis() - last_warn_ms >= 500) {
          last_warn_ms = millis();
          Serial.print("WARN:NOISE,");
          Serial.print(el);
          Serial.print(",");
          Serial.println(er);
        }
      }
    } else if (labs32(min(abs_el, abs_er)) >= target_ticks) {
      stopMotors();
      Serial.println("ACK");
    } else if (millis() - move_start_ms > MOVE_TIMEOUT_MS) {
      stopMotors();
      Serial.println("ERR:TIMEOUT");
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
