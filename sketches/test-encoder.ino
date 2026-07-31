// ===== ENCODER TEST FIRMWARE =====
// Goal: verify encoder counts are correct and stable

// -------- PIN CONFIG --------
// ⚠️ CHANGE THESE to match your wiring
const int LEFT_ENC_PIN  = 2;   // must be interrupt pin on UNO
const int RIGHT_ENC_PIN = 3;   // must be interrupt pin on UNO

const int LEFT_ENC_PIN_B  = 4;   // must be interrupt pin on UNO
const int RIGHT_ENC_PIN_B = 8;   // must be interrupt pin on UNO

// -------- ENCODER COUNTS --------
volatile long left_count  = 0;
volatile long right_count = 0;

volatile long left_count_b  = 0;
volatile long right_count_b = 0;

// -------- ISR (Interrupt Service Routines) --------
void leftEncoderISR() {
  left_count++;
}

void rightEncoderISR() {
  right_count++;
}

void leftEncoderBISR() {
  left_count_b++;
}

void rightEncoderBISR() {
  right_count_b++;
}

void setup() {
  Serial.begin(115200);

  // Configure encoder pins
  pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);

  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightEncoderISR, RISING);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), leftEncoderBISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), rightEncoderBISR, RISING);

  Serial.println("=== ENCODER TEST START ===");
}

void loop() {
  static unsigned long lastPrint = 0;

  // Print every 100 ms (stable, readable)
  if (millis() - lastPrint > 100) {
    lastPrint = millis();

    // Copy safely (VERY IMPORTANT)
    noInterrupts();
    long l = left_count;
    long r = right_count;

    long lb = left_count_b;
    long rb = right_count_b;

    interrupts();

    Serial.print("ENC-");
    Serial.print(l);
    Serial.print(":");
    Serial.print(lb);
    Serial.print(",");
    Serial.print(r);
    Serial.print(":");
    Serial.println(rb);
  }
}