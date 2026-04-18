/*
 * mecanum_test.ino — Full Movement Tester for Osoyoo Mecanum Car
 * Arduino Mega  •  9600 baud
 *
 * TWO MODES:
 *   1) AUTO-CYCLE: on boot, runs each direction for 2 seconds
 *      so you can set the car on a box and watch the wheels.
 *   2) SERIAL COMMANDS: after the auto-cycle, send single chars
 *      from the Serial Monitor (or a Python script) to drive manually.
 *
 * Commands:
 *   'F' → Forward        'B' → Backward
 *   'L' → Strafe Left    'R' → Strafe Right
 *   'Z' → Rotate Left    'X' → Rotate Right
 *   'S' → Stop
 *
 * Wiring (Osoyoo Mecanum):
 *   Front Right (FR): PWM = D9,  DIR = 22, 24
 *   Front Left  (FL): PWM = D10, DIR = 26, 28
 *   Rear Right  (RR): PWM = D11, DIR = 5,  6
 *   Rear Left   (RL): PWM = D12, DIR = 7,  8
 *
 * !! IMPORTANT !!
 *   Prop the car up on a box before uploading so wheels spin freely.
 *   If a direction is wrong, swap the two DIR pins for that motor
 *   (e.g. swap 22↔24) or flip the LOW/HIGH in the code below.
 */

// ── Pin definitions ──────────────────────────────────────────────────────────
// Front Right
#define FR_PWM   9
#define FR_DIR1  22
#define FR_DIR2  24

// Front Left
#define FL_PWM   10
#define FL_DIR1  26
#define FL_DIR2  28

// Rear Right
#define RR_PWM   11
#define RR_DIR1  5
#define RR_DIR2  6

// Rear Left
#define RL_PWM   12
#define RL_DIR1  7
#define RL_DIR2  8

// Speed for testing (keep low-ish so it's safe on a bench)
#define TEST_SPEED 130

// How long each direction runs during auto-cycle (ms)
#define TEST_DURATION 2000


// ── Per-wheel helpers ────────────────────────────────────────────────────────

void FR_fwd(int s) { digitalWrite(FR_DIR1, LOW);  digitalWrite(FR_DIR2, HIGH); analogWrite(FR_PWM, s); }
void FR_rev(int s) { digitalWrite(FR_DIR1, HIGH); digitalWrite(FR_DIR2, LOW);  analogWrite(FR_PWM, s); }

void FL_fwd(int s) { digitalWrite(FL_DIR1, LOW);  digitalWrite(FL_DIR2, HIGH); analogWrite(FL_PWM, s); }
void FL_rev(int s) { digitalWrite(FL_DIR1, HIGH); digitalWrite(FL_DIR2, LOW);  analogWrite(FL_PWM, s); }

void RR_fwd(int s) { digitalWrite(RR_DIR1, LOW);  digitalWrite(RR_DIR2, HIGH); analogWrite(RR_PWM, s); }
void RR_rev(int s) { digitalWrite(RR_DIR1, HIGH); digitalWrite(RR_DIR2, LOW);  analogWrite(RR_PWM, s); }

void RL_fwd(int s) { digitalWrite(RL_DIR1, LOW);  digitalWrite(RL_DIR2, HIGH); analogWrite(RL_PWM, s); }
void RL_rev(int s) { digitalWrite(RL_DIR1, HIGH); digitalWrite(RL_DIR2, LOW);  analogWrite(RL_PWM, s); }

void stop_all() {
  analogWrite(FR_PWM, 0);
  analogWrite(FL_PWM, 0);
  analogWrite(RR_PWM, 0);
  analogWrite(RL_PWM, 0);
  Serial.println(F(">> STOP"));
}

// ══════════════════════════════════════════════════════════════════════════════
//  MECANUM MOVEMENT COMMANDS
//
//  Wheel direction cheat-sheet (top-down view):
//
//      FL ──── FR          FWD:  all forward
//      |      |            BWD:  all reverse
//      RL ──── RR          STR-L: FL↓ FR↑ RL↑ RR↓
//                          STR-R: FL↑ FR↓ RL↓ RR↑
//                          ROT-L: FL↓ FR↑ RL↓ RR↑
//                          ROT-R: FL↑ FR↓ RL↑ RR↓
// ══════════════════════════════════════════════════════════════════════════════

void drive_forward(int s) {
  FL_fwd(s); FR_fwd(s); RL_fwd(s); RR_fwd(s);
  Serial.println(F(">> FORWARD"));
}

void drive_backward(int s) {
  FL_rev(s); FR_rev(s); RL_rev(s); RR_rev(s);
  Serial.println(F(">> BACKWARD"));
}

void strafe_left(int s) {
  FL_rev(s); FR_fwd(s); RL_fwd(s); RR_rev(s);
  Serial.println(F(">> STRAFE LEFT"));
}

void strafe_right(int s) {
  FL_fwd(s); FR_rev(s); RL_rev(s); RR_fwd(s);
  Serial.println(F(">> STRAFE RIGHT"));
}

void rotate_left(int s) {
  FL_rev(s); FR_fwd(s); RL_rev(s); RR_fwd(s);
  Serial.println(F(">> ROTATE LEFT"));
}

void rotate_right(int s) {
  FL_fwd(s); FR_rev(s); RL_fwd(s); RR_rev(s);
  Serial.println(F(">> ROTATE RIGHT"));
}


// ── GPIO init ────────────────────────────────────────────────────────────────
void init_GPIO() {
  int pins[] = {
    FR_DIR1, FR_DIR2, FR_PWM,
    FL_DIR1, FL_DIR2, FL_PWM,
    RR_DIR1, RR_DIR2, RR_PWM,
    RL_DIR1, RL_DIR2, RL_PWM
  };
  for (int i = 0; i < 12; i++) pinMode(pins[i], OUTPUT);
  stop_all();
}


// ══════════════════════════════════════════════════════════════════════════════
//  SETUP — auto-cycle through every direction
// ══════════════════════════════════════════════════════════════════════════════
void setup() {
  init_GPIO();
  Serial.begin(9600);

  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("  MECANUM MOVEMENT TEST — Osoyoo / Mega"));
  Serial.println(F("========================================"));
  Serial.println(F("Starting auto-cycle in 3 seconds..."));
  Serial.println(F("(Prop car on a box so wheels spin free!)"));
  Serial.println(F(""));
  delay(3000);

  // ── Test each direction ────────────────────────────────────────────────

  Serial.println(F("--- TEST 1/6: FORWARD ---"));
  drive_forward(TEST_SPEED);
  delay(TEST_DURATION);
  stop_all();
  delay(1000);

  Serial.println(F("--- TEST 2/6: BACKWARD ---"));
  drive_backward(TEST_SPEED);
  delay(TEST_DURATION);
  stop_all();
  delay(1000);

  Serial.println(F("--- TEST 3/6: STRAFE LEFT ---"));
  strafe_left(TEST_SPEED);
  delay(TEST_DURATION);
  stop_all();
  delay(1000);

  Serial.println(F("--- TEST 4/6: STRAFE RIGHT ---"));
  strafe_right(TEST_SPEED);
  delay(TEST_DURATION);
  stop_all();
  delay(1000);

  Serial.println(F("--- TEST 5/6: ROTATE LEFT ---"));
  rotate_left(TEST_SPEED);
  delay(TEST_DURATION);
  stop_all();
  delay(1000);

  Serial.println(F("--- TEST 6/6: ROTATE RIGHT ---"));
  rotate_right(TEST_SPEED);
  delay(TEST_DURATION);
  stop_all();
  delay(1000);

  // ── Done ───────────────────────────────────────────────────────────────
  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("  AUTO-CYCLE COMPLETE"));
  Serial.println(F("========================================"));
  Serial.println(F("Now in manual mode. Send commands:"));
  Serial.println(F("  F = Forward     B = Backward"));
  Serial.println(F("  L = Strafe Left R = Strafe Right"));
  Serial.println(F("  Z = Rotate Left X = Rotate Right"));
  Serial.println(F("  S = Stop"));
  Serial.println(F(""));
}


// ══════════════════════════════════════════════════════════════════════════════
//  LOOP — manual serial control
// ══════════════════════════════════════════════════════════════════════════════
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'F': drive_forward(TEST_SPEED);  break;
      case 'B': drive_backward(TEST_SPEED); break;
      case 'L': strafe_left(TEST_SPEED);    break;
      case 'R': strafe_right(TEST_SPEED);   break;
      case 'Z': rotate_left(TEST_SPEED);    break;
      case 'X': rotate_right(TEST_SPEED);   break;
      case 'S': stop_all();                 break;
      // ignore \r, \n, anything else
    }
  }
}
