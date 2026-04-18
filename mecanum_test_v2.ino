/*
 * mecanum_test_v2.ino — Full Diagnostic + 8-Direction Mecanum Test
 * Arduino Mega  •  9600 baud  •  Osoyoo Mecanum Chassis
 *
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  PHASE 1: Individual wheel test (find which motors are dead)   ║
 * ║  PHASE 2: 8-direction auto-cycle (every 45°)                  ║
 * ║  PHASE 3: Manual serial control                                ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * PROP THE CAR ON A BOX before uploading so wheels spin free!
 *
 * Serial commands (after auto-cycle):
 *
 *        Numpad layout:            Also:
 *     7=FwdLeft  8=Fwd  9=FwdRight    Z = Rotate Left (CCW)
 *     4=Left     5=Stop 6=Right       X = Rotate Right (CW)
 *     1=BkLeft   2=Bk   3=BkRight     S = Stop
 *
 * Wiring (Osoyoo Mecanum):
 *   Front Left  (FL): PWM=D10, DIR=26,28
 *   Front Right (FR): PWM=D9,  DIR=22,24
 *   Rear  Left  (RL): PWM=D12, DIR=7,8
 *   Rear  Right (RR): PWM=D11, DIR=5,6
 */

// ═══════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════

// Front Left
#define FL_PWM   10
#define FL_DIR1  26
#define FL_DIR2  28

// Front Right
#define FR_PWM   9
#define FR_DIR1  22
#define FR_DIR2  24

// Rear Left
#define RL_PWM   12
#define RL_DIR1  7
#define RL_DIR2  8

// Rear Right
#define RR_PWM   11
#define RR_DIR1  5
#define RR_DIR2  6

#define TEST_SPEED  150       // 0–255, for auto-cycle & manual
#define DIAG_SPEED  120       // slightly slower for single-wheel test
#define STEP_TIME   2000      // ms per auto-cycle step
#define PAUSE_TIME  800       // ms pause between steps


// ═══════════════════════════════════════════════════════════════════
//  PER-WHEEL PRIMITIVES
// ═══════════════════════════════════════════════════════════════════

void FL_fwd(int s) { digitalWrite(FL_DIR1, LOW);  digitalWrite(FL_DIR2, HIGH); analogWrite(FL_PWM, s); }
void FL_rev(int s) { digitalWrite(FL_DIR1, HIGH); digitalWrite(FL_DIR2, LOW);  analogWrite(FL_PWM, s); }
void FL_off()      { analogWrite(FL_PWM, 0); }

void FR_fwd(int s) { digitalWrite(FR_DIR1, LOW);  digitalWrite(FR_DIR2, HIGH); analogWrite(FR_PWM, s); }
void FR_rev(int s) { digitalWrite(FR_DIR1, HIGH); digitalWrite(FR_DIR2, LOW);  analogWrite(FR_PWM, s); }
void FR_off()      { analogWrite(FR_PWM, 0); }

void RL_fwd(int s) { digitalWrite(RL_DIR1, LOW);  digitalWrite(RL_DIR2, HIGH); analogWrite(RL_PWM, s); }
void RL_rev(int s) { digitalWrite(RL_DIR1, HIGH); digitalWrite(RL_DIR2, LOW);  analogWrite(RL_PWM, s); }
void RL_off()      { analogWrite(RL_PWM, 0); }

void RR_fwd(int s) { digitalWrite(RR_DIR1, LOW);  digitalWrite(RR_DIR2, HIGH); analogWrite(RR_PWM, s); }
void RR_rev(int s) { digitalWrite(RR_DIR1, HIGH); digitalWrite(RR_DIR2, LOW);  analogWrite(RR_PWM, s); }
void RR_off()      { analogWrite(RR_PWM, 0); }

void stop_all() {
  FL_off(); FR_off(); RL_off(); RR_off();
  Serial.println(F(">> STOP"));
}


// ═══════════════════════════════════════════════════════════════════
//  8-DIRECTION MOVEMENT  (every 45°)
//
//  Compass rose — top of car is "North" (forward):
//
//          0° FWD
//    315° ╱       ╲ 45°
//   270° ←    ☉    → 90°
//    225° ╲       ╱ 135°
//         180° BWD
//
//  Mecanum wheel physics:
//   ┌────────────────────────────────────────────────────────┐
//   │ Direction     │  FL  │  FR  │  RL  │  RR  │ Angle     │
//   ├────────────────────────────────────────────────────────┤
//   │ Forward       │  ↑   │  ↑   │  ↑   │  ↑   │   0°     │
//   │ Fwd-Right     │  ↑   │  ×   │  ×   │  ↑   │  45°     │
//   │ Strafe Right  │  ↑   │  ↓   │  ↓   │  ↑   │  90°     │
//   │ Back-Right    │  ×   │  ↓   │  ↓   │  ×   │ 135°     │
//   │ Backward      │  ↓   │  ↓   │  ↓   │  ↓   │ 180°     │
//   │ Back-Left     │  ↓   │  ×   │  ×   │  ↓   │ 225°     │
//   │ Strafe Left   │  ↓   │  ↑   │  ↑   │  ↓   │ 270°     │
//   │ Fwd-Left      │  ×   │  ↑   │  ↑   │  ×   │ 315°     │
//   └────────────────────────────────────────────────────────┘
//    ↑ = forward   ↓ = reverse   × = stopped
//
// ═══════════════════════════════════════════════════════════════════

// --- 0°  FORWARD ---
void move_forward(int s) {
  FL_fwd(s); FR_fwd(s);
  RL_fwd(s); RR_fwd(s);
  Serial.println(F(">> 0° FORWARD  (FL↑ FR↑ RL↑ RR↑)"));
}

// --- 45°  FORWARD-RIGHT DIAGONAL ---
void move_fwd_right(int s) {
  FL_fwd(s); FR_off();
  RL_off();  RR_fwd(s);
  Serial.println(F(">> 45° FWD-RIGHT  (FL↑ FR× RL× RR↑)"));
}

// --- 90°  STRAFE RIGHT ---
void move_right(int s) {
  FL_fwd(s); FR_rev(s);
  RL_rev(s); RR_fwd(s);
  Serial.println(F(">> 90° STRAFE RIGHT  (FL↑ FR↓ RL↓ RR↑)"));
}

// --- 135°  BACKWARD-RIGHT DIAGONAL ---
void move_bk_right(int s) {
  FL_off();  FR_rev(s);
  RL_rev(s); RR_off();
  Serial.println(F(">> 135° BK-RIGHT  (FL× FR↓ RL↓ RR×)"));
}

// --- 180°  BACKWARD ---
void move_backward(int s) {
  FL_rev(s); FR_rev(s);
  RL_rev(s); RR_rev(s);
  Serial.println(F(">> 180° BACKWARD  (FL↓ FR↓ RL↓ RR↓)"));
}

// --- 225°  BACKWARD-LEFT DIAGONAL ---
void move_bk_left(int s) {
  FL_rev(s); FR_off();
  RL_off();  RR_rev(s);
  Serial.println(F(">> 225° BK-LEFT  (FL↓ FR× RL× RR↓)"));
}

// --- 270°  STRAFE LEFT ---
void move_left(int s) {
  FL_rev(s); FR_fwd(s);
  RL_fwd(s); RR_rev(s);
  Serial.println(F(">> 270° STRAFE LEFT  (FL↓ FR↑ RL↑ RR↓)"));
}

// --- 315°  FORWARD-LEFT DIAGONAL ---
void move_fwd_left(int s) {
  FL_off();  FR_fwd(s);
  RL_fwd(s); RR_off();
  Serial.println(F(">> 315° FWD-LEFT  (FL× FR↑ RL↑ RR×)"));
}

// --- ROTATIONS (spin in place) ---
void rotate_left(int s) {
  FL_rev(s); FR_fwd(s);
  RL_rev(s); RR_fwd(s);
  Serial.println(F(">> ROTATE LEFT/CCW  (FL↓ FR↑ RL↓ RR↑)"));
}

void rotate_right(int s) {
  FL_fwd(s); FR_rev(s);
  RL_fwd(s); RR_rev(s);
  Serial.println(F(">> ROTATE RIGHT/CW  (FL↑ FR↓ RL↑ RR↓)"));
}


// ═══════════════════════════════════════════════════════════════════
//  GPIO INIT
// ═══════════════════════════════════════════════════════════════════
void init_GPIO() {
  int pins[] = {
    FL_DIR1, FL_DIR2, FL_PWM,
    FR_DIR1, FR_DIR2, FR_PWM,
    RL_DIR1, RL_DIR2, RL_PWM,
    RR_DIR1, RR_DIR2, RR_PWM
  };
  for (int i = 0; i < 12; i++) pinMode(pins[i], OUTPUT);
  stop_all();
}


// helper: run a step in the auto-cycle
void test_step(const char* label, void (*action)(int), int spd) {
  Serial.println(label);
  action(spd);
  delay(STEP_TIME);
  stop_all();
  delay(PAUSE_TIME);
}


// ═══════════════════════════════════════════════════════════════════
//  SETUP — diagnostics then auto-cycle
// ═══════════════════════════════════════════════════════════════════
void setup() {
  init_GPIO();
  Serial.begin(9600);

  Serial.println(F(""));
  Serial.println(F("╔═══════════════════════════════════════════╗"));
  Serial.println(F("║  MECANUM FULL TEST v2 — Osoyoo / Mega    ║"));
  Serial.println(F("╚═══════════════════════════════════════════╝"));
  Serial.println(F(""));


  // ────────────────────────────────────────────────────────────────
  //  PHASE 1: Individual wheel test
  //  Watch which wheels spin. If a wheel doesn't move here,
  //  it's a WIRING issue — not a code issue.
  // ────────────────────────────────────────────────────────────────
  Serial.println(F("══ PHASE 1: INDIVIDUAL WHEEL TEST ══"));
  Serial.println(F("Watch each wheel — if one doesn't spin,"));
  Serial.println(F("check that motor's wiring / driver channel."));
  Serial.println(F("Starting in 3 sec...\n"));
  delay(3000);

  Serial.println(F("  [1/4] FRONT LEFT only..."));
  FL_fwd(DIAG_SPEED);
  delay(1500);
  FL_off();
  delay(600);

  Serial.println(F("  [2/4] FRONT RIGHT only..."));
  FR_fwd(DIAG_SPEED);
  delay(1500);
  FR_off();
  delay(600);

  Serial.println(F("  [3/4] REAR LEFT only..."));
  RL_fwd(DIAG_SPEED);
  delay(1500);
  RL_off();
  delay(600);

  Serial.println(F("  [4/4] REAR RIGHT only..."));
  RR_fwd(DIAG_SPEED);
  delay(1500);
  RR_off();
  delay(600);

  Serial.println(F(""));
  Serial.println(F("Did all 4 wheels spin?"));
  Serial.println(F("  YES → great, moving to Phase 2"));
  Serial.println(F("  NO  → check wiring on the dead motor(s)"));
  Serial.println(F("         PWM pins: FL=D10, FR=D9, RL=D12, RR=D11"));
  Serial.println(F("         DIR pins: FL=26/28, FR=22/24, RL=7/8, RR=5/6"));
  Serial.println(F(""));
  delay(3000);


  // ────────────────────────────────────────────────────────────────
  //  PHASE 2: 8-direction auto-cycle (every 45°) + rotations
  // ────────────────────────────────────────────────────────────────
  Serial.println(F("══ PHASE 2: 8-DIRECTION AUTO-CYCLE ══\n"));

  test_step("--- 0° FORWARD ---",        move_forward,   TEST_SPEED);
  test_step("--- 45° FWD-RIGHT ---",      move_fwd_right, TEST_SPEED);
  test_step("--- 90° STRAFE RIGHT ---",   move_right,     TEST_SPEED);
  test_step("--- 135° BACK-RIGHT ---",    move_bk_right,  TEST_SPEED);
  test_step("--- 180° BACKWARD ---",      move_backward,  TEST_SPEED);
  test_step("--- 225° BACK-LEFT ---",     move_bk_left,   TEST_SPEED);
  test_step("--- 270° STRAFE LEFT ---",   move_left,      TEST_SPEED);
  test_step("--- 315° FWD-LEFT ---",      move_fwd_left,  TEST_SPEED);
  test_step("--- ROTATE LEFT (CCW) ---",  rotate_left,    TEST_SPEED);
  test_step("--- ROTATE RIGHT (CW) ---",  rotate_right,   TEST_SPEED);


  // ────────────────────────────────────────────────────────────────
  //  PHASE 3: Manual control
  // ────────────────────────────────────────────────────────────────
  Serial.println(F(""));
  Serial.println(F("╔═══════════════════════════════════════════╗"));
  Serial.println(F("║  AUTO-CYCLE DONE — MANUAL MODE ACTIVE    ║"));
  Serial.println(F("╠═══════════════════════════════════════════╣"));
  Serial.println(F("║  Numpad-style commands:                   ║"));
  Serial.println(F("║     7 = Fwd-Left    8 = Fwd   9 = Fwd-Rt ║"));
  Serial.println(F("║     4 = Left        5 = Stop  6 = Right   ║"));
  Serial.println(F("║     1 = Bk-Left     2 = Bk    3 = Bk-Rt  ║"));
  Serial.println(F("║                                           ║"));
  Serial.println(F("║     Z = Rotate Left   X = Rotate Right    ║"));
  Serial.println(F("║     S = Stop                              ║"));
  Serial.println(F("╚═══════════════════════════════════════════╝"));
  Serial.println(F(""));
}


// ═══════════════════════════════════════════════════════════════════
//  LOOP — manual serial control
// ═══════════════════════════════════════════════════════════════════
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      // numpad directions
      case '8': move_forward(TEST_SPEED);   break;   //   0°
      case '9': move_fwd_right(TEST_SPEED); break;   //  45°
      case '6': move_right(TEST_SPEED);     break;   //  90°
      case '3': move_bk_right(TEST_SPEED);  break;   // 135°
      case '2': move_backward(TEST_SPEED);  break;   // 180°
      case '1': move_bk_left(TEST_SPEED);   break;   // 225°
      case '4': move_left(TEST_SPEED);      break;   // 270°
      case '7': move_fwd_left(TEST_SPEED);  break;   // 315°

      // rotations
      case 'Z': rotate_left(TEST_SPEED);    break;
      case 'X': rotate_right(TEST_SPEED);   break;

      // stop
      case '5': // fallthrough
      case 'S': stop_all();                 break;
    }
  }
}
