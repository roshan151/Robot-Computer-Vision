// ===== MOTOR DRIVER DIAGNOSTIC =====
// Bypasses all serial protocol. Blinks each motor on/off so you can
// immediately see whether the motor driver responds to the Arduino pins.
//
// Expected: each motor spins for 1 s, stops for 1 s, alternating.
// If nothing moves: check ENA/ENB jumpers, motor Vin, and IN wiring.
//
// Adjust the pin numbers below to match your actual wiring if needed.

const int L_ENA = 5;   // PWM speed — remove L298N ENA jumper!
const int L_IN1 = 6;
const int L_IN2 = 7;

const int R_ENA = 9;   // PWM speed — remove L298N ENB jumper!
const int R_IN1 = 10;
const int R_IN2 = 11;

const int TEST_PWM = 180;   // 0–255; increase if motors barely move

void motorForward(int ena, int in1, int in2, int pwm) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena, pwm);
}

void motorStop(int ena, int in1, int in2) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(ena, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(L_ENA, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_ENA, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);

  // Start stopped
  motorStop(L_ENA, L_IN1, L_IN2);
  motorStop(R_ENA, R_IN1, R_IN2);

  Serial.println("Motor diagnostic start. Watch for movement every 1 s.");
}

void loop() {
  Serial.println("LEFT  motor ON");
  motorForward(L_ENA, L_IN1, L_IN2, TEST_PWM);
  delay(1000);

  Serial.println("LEFT  motor OFF");
  motorStop(L_ENA, L_IN1, L_IN2);
  delay(500);

  Serial.println("RIGHT motor ON");
  motorForward(R_ENA, R_IN1, R_IN2, TEST_PWM);
  delay(1000);

  Serial.println("RIGHT motor OFF");
  motorStop(R_ENA, R_IN1, R_IN2);
  delay(500);

  Serial.println("BOTH  motors ON");
  motorForward(L_ENA, L_IN1, L_IN2, TEST_PWM);
  motorForward(R_ENA, R_IN1, R_IN2, TEST_PWM);
  delay(1000);

  Serial.println("BOTH  motors OFF");
  motorStop(L_ENA, L_IN1, L_IN2);
  motorStop(R_ENA, R_IN1, R_IN2);
  delay(1000);
}
