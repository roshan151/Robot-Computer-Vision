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
 *  4. Sync PID — proportional correction keeps wheels in step, with bounded
 *     authority so it can never command a wheel to 0 (which would open the
 *     loop) or above the requested speed (which browned out the board).
 *  5. Per-wheel signed encoder counting (enc_dir_l / enc_dir_r), so turns —
 *     where the wheels counter-rotate — are measured correctly.
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

// Bounds on the sync correction.  These are NOT cosmetic — without them a
// persistent encoder bias (one channel under-counting) makes the P term grow
// without limit until it saturates:
//   lagging wheel  → base + 200 → motorWrite() clamps to 255 → analogWrite()
//                    turns that into digitalWrite(HIGH) = 100 % duty, full
//                    battery voltage, no PWM switching, huge current step.
//   leading wheel  → clamped to 0 → coasts → its encoder STOPS COUNTING →
//                    diff can never shrink → the controller latches there.
// That combination is what browned out the Arduino mid-reverse (BOOT:2,4).
// So: cap the diff fed to the P term, cap the correction as a fraction of the
// commanded speed, and never let a wheel be commanded to 0 or above PWM_MAX.
#define SYNC_DIFF_MAX       300   // ticks of error the P term may see
#define SYNC_AUTHORITY_PCT  50    // max correction, as % of the base speed
#define SYNC_FLOOR_PCT      35    // a wheel is never driven below this % of base

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
// Each wheel gets its OWN direction sign.  A single shared enc_dir was wrong
// for turns: during 'L' and 'R' the wheels counter-rotate, so one of the two
// counters ran the wrong way and the sync correction was applied to the right
// wheel with an inverted sign — the controller pushed the wheels apart instead
// of together on every turn.
//
// Convention: enc_left/enc_right are signed the way the wheel physically turns
// (so a reverse move still reports negative counts to the Pi), while
// enc_left * enc_dir_l is the wheel's PROGRESS along the commanded direction
// and is always >= 0.  All control decisions use progress; only telemetry uses
// the raw signed value.
volatile long enc_left  = 0;
volatile long enc_right = 0;
volatile int  enc_dir_l = 1;
volatile int  enc_dir_r = 1;

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

// ---- Open-loop (intent) speed ----
// Default PWM used by the bare F/B/L/R commands.  V:<pwm> updates it so a
// speed override survives the brake dead-time of a direction flip — the old
// code only rewrote the live target_pwm_*, which are zero while braking, so
// the override was silently dropped and the robot drove at the default.
#define OPEN_LOOP_PWM  150
int open_loop_speed = OPEN_LOOP_PWM;

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
  enc_left += enc_dir_l;
}
void isr_right() {
  unsigned long now = micros();
  if (now - last_right_us < MIN_PULSE_US) return;
  last_right_us = now;
  enc_right += enc_dir_r;
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

  // Each encoder counts in the direction ITS OWN wheel is commanded to turn.
  // For 'L' and 'R' the two wheels counter-rotate, so these signs differ.
  int left_sign, right_sign;
  wheelSigns(dir, left_sign, right_sign);

  // Set encoder directions, then atomically reset counts.
  // Also reset ISR debounce timestamps so the first real edge of the new
  // move is not rejected because it happens to fall within MIN_PULSE_US
  // of a stale timestamp from the previous move.
  noInterrupts();
  enc_dir_l     = left_sign;
  enc_dir_r     = right_sign;
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
  target_pwm_left  = left_sign  * speed;
  target_pwm_right = right_sign * speed;

  noise_corrections = 0;

  last_ramp_ms = millis();
  // ACK is sent by loop() once the encoder target is reached.
}

// --------------------------------------------------
// beginOpenLoop — start (or change) an untimed intent move.
//
// No encoder target and no sync PID, but the encoder DIRECTIONS and
// current_dir are still maintained so that counts stay meaningful and the
// next direction flip gets its brake dead-time.
// --------------------------------------------------
void beginOpenLoop(char dir) {
  int left_sign, right_sign;
  wheelSigns(dir, left_sign, right_sign);

  coastMotors();          // release the brake; loop() ramps up from zero
  current_dir = dir;
  move_active = false;

  noInterrupts();
  enc_dir_l = left_sign;
  enc_dir_r = right_sign;
  interrupts();

  target_pwm_left  = left_sign  * open_loop_speed;
  target_pwm_right = right_sign * open_loop_speed;
  last_ramp_ms     = millis();
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
  // ACK immediately; no encoder target, but the encoder DIRECTION and
  // current_dir are still updated.  Without that, counts kept accumulating
  // with whatever sign the last M: move left behind (which is why the timed
  // test reported positive deltas while reversing), and isReversal() compared
  // against a stale direction so an F→B flip got no brake dead-time.
  if (strcmp(cmd, "F") == 0 || strcmp(cmd, "B") == 0 ||
      strcmp(cmd, "L") == 0 || strcmp(cmd, "R") == 0) {
    char dir = cmd[0];
    move_active = false;

    // A direction flip still needs the brake dead-time.  pending_ticks == 0
    // marks this as an open-loop start so loop() applies PWM without arming
    // an encoder target.  Non-blocking, so the serial port stays serviced.
    if (isReversal(current_dir, dir)) {
      brakeMotors();
      brake_until_ms = millis() + BRAKE_MS_REVERSE;
      pending_dir    = dir;
      pending_speed  = open_loop_speed;
      pending_ticks  = 0;
      brake_pending  = true;
    } else {
      beginOpenLoop(dir);
    }
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
      open_loop_speed = v;              // survives a pending brake dead-time
      if (brake_pending && pending_ticks == 0) pending_speed = v;
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
    if (pending_ticks > 0) {
      beginDrive(pending_dir, pending_speed, pending_ticks);
    } else {
      // pending_ticks == 0 → open-loop intent move armed by F/B/L/R.
      open_loop_speed = pending_speed;
      beginOpenLoop(pending_dir);
    }
  }

  // ---- Timed PWM ramp + sync PID ----
  unsigned long now = millis();
  if (now - last_ramp_ms >= RAMP_MS) {
    last_ramp_ms = now;

    int tl = target_pwm_left;
    int tr = target_pwm_right;

    if (move_active && tl != 0 && tr != 0) {
      noInterrupts();
      long el = enc_left;
      long er = enc_right;
      interrupts();

      // Progress along each wheel's own commanded direction — always >= 0,
      // and directly comparable between the wheels for F, B, L and R alike.
      long pl = el * (long)enc_dir_l;
      long pr = er * (long)enc_dir_r;

      long diff = pl - pr;            // > 0 ⇒ left wheel is ahead
      if (diff >  SYNC_DIFF_MAX) diff =  SYNC_DIFF_MAX;
      if (diff < -SYNC_DIFF_MAX) diff = -SYNC_DIFF_MAX;

      // Work in magnitudes, then re-apply each wheel's commanded sign.
      int base = (target_pwm_left < 0) ? -target_pwm_left : target_pwm_left;
      int lim  = (int)((long)base * SYNC_AUTHORITY_PCT / 100);
      int corr = (int)(SYNC_KP * (float)diff);
      if (corr >  lim) corr =  lim;
      if (corr < -lim) corr = -lim;

      int floor_pwm = (int)((long)base * SYNC_FLOOR_PCT / 100);
      int mag_l = base - corr;        // left ahead  → slow left down
      int mag_r = base + corr;        // left ahead  → speed right up
      if (mag_l < floor_pwm) mag_l = floor_pwm;
      if (mag_r < floor_pwm) mag_r = floor_pwm;
      if (mag_l > PWM_MAX)   mag_l = PWM_MAX;
      if (mag_r > PWM_MAX)   mag_r = PWM_MAX;

      // The floor is the important half of this clamp: a wheel commanded to
      // 0 coasts, stops producing encoder edges, and therefore freezes `diff`
      // — which pins the other wheel at full correction until the battery or
      // the Arduino gives out.  Never let the loop open itself.
      tl = (target_pwm_left  < 0) ? -mag_l : mag_l;
      tr = (target_pwm_right < 0) ? -mag_r : mag_r;
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
  // Noise sanity: if the two wheels diverge wildly mid-move, one channel
  // is counting noise.  A burst only ever ADDS counts, so the wheel with
  // the SMALLER magnitude is the trustworthy one — snap the bad counter
  // back to it and keep driving.  Aborting the whole move on a single
  // glitch is what made an otherwise healthy turn fail.  Only give up if
  // the corruption keeps recurring.
  if (move_active) {
    noInterrupts();
    long el = enc_left;
    long er = enc_right;
    interrupts();

    // Progress along each wheel's commanded direction.  A negative value
    // means the counter ran backwards, which can only be noise (or a wheel
    // being dragged) — treat it as no progress rather than as travel.
    long pl = el * (long)enc_dir_l;
    long pr = er * (long)enc_dir_r;
    if (pl < 0) pl = 0;
    if (pr < 0) pr = 0;

    // Detect runaway noise: one wheel reports >>10x the other once both
    // have moved enough that the ratio is meaningful (>50 ticks).  This
    // catches the "left encoder spewing 10^9 counts" failure mode early.
    bool noise_runaway = false;
    if (pl > 50 && pr > 50) {
      if (pl > 10 * pr || pr > 10 * pl) {
        noise_runaway = true;
      }
    } else if (pl > 10000 && pr < 10) {
      noise_runaway = true;
    } else if (pr > 10000 && pl < 10) {
      noise_runaway = true;
    }

    if (noise_runaway) {
      noise_corrections++;
      if (noise_corrections > MAX_NOISE_CORRECTIONS) {
        stopMotors();
        Serial.print("ERR:NOISE,");
        Serial.print(el);
        Serial.print(",");
        Serial.println(er);
      } else {
        // Noise only ever ADDS counts, so the SMALLER reading is the
        // trustworthy one.  Snap only the corrupted channel back to it and
        // leave the healthy counter alone.  The old code overwrote BOTH,
        // so a single sample of "0, 38657" reset the good channel to zero
        // too — throwing away all real progress and making the move
        // impossible to finish.
        long good = (pl < pr) ? pl : pr;
        noInterrupts();
        if (pl > pr) enc_left  = good * (long)enc_dir_l;
        else         enc_right = good * (long)enc_dir_r;
        interrupts();
        // Tell the Pi it happened without failing the move.
        Serial.print("WARN:NOISE,");
        Serial.print(el);
        Serial.print(",");
        Serial.println(er);
      }
    } else if (pl >= target_ticks && pr >= target_ticks) {
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
