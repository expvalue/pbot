/*
 * pbot_v5.ino — Fire bot with auto-staggered extinguisher servos
 * Arduino Mega  •  9600 baud  •  Osoyoo Mecanum
 *
 * Commands from firebot_v5.py:
 *   'F' → drive forward
 *   'S' → stop all motors
 *   'E' → fire servo 1 → wait 750ms → fire servo 2 (all on Arduino)
 *
 * Servos not attached until triggered — no startup twitch.
 * Servos: D3 = extinguisher 1, D4 = extinguisher 2
 * SAFETY: auto-stops if no command for 1 second
 */

#include <Servo.h>

#define FL_PWM  10
#define FL_DIR1 26
#define FL_DIR2 28

#define FR_PWM  9
#define FR_DIR1 22
#define FR_DIR2 24

#define RL_PWM  12
#define RL_DIR1 7
#define RL_DIR2 8

#define RR_PWM  11
#define RR_DIR1 5
#define RR_DIR2 6

#define SERVO1_PIN 3
#define SERVO2_PIN 4

#define DRIVE_SPEED 150
#define WATCHDOG_MS 1000

Servo servo1;
Servo servo2;

unsigned long lastCmdTime   = 0;
bool          moving        = false;
bool          extinguished  = false;

void FL_fwd(int s) { digitalWrite(FL_DIR1, HIGH); digitalWrite(FL_DIR2, LOW);  analogWrite(FL_PWM, s); }
void FR_fwd(int s) { digitalWrite(FR_DIR1, HIGH); digitalWrite(FR_DIR2, LOW);  analogWrite(FR_PWM, s); }
void RL_fwd(int s) { digitalWrite(RL_DIR1, HIGH); digitalWrite(RL_DIR2, LOW);  analogWrite(RL_PWM, s); }
void RR_fwd(int s) { digitalWrite(RR_DIR1, HIGH); digitalWrite(RR_DIR2, LOW);  analogWrite(RR_PWM, s); }

void drive_forward(int s) {
  FL_fwd(s); FR_fwd(s); RL_fwd(s); RR_fwd(s);
  moving = true;
}

void stop_all() {
  analogWrite(FL_PWM, 0);
  analogWrite(FR_PWM, 0);
  analogWrite(RL_PWM, 0);
  analogWrite(RR_PWM, 0);
  moving = false;
}

void fire_extinguishers() {
  if (extinguished) return;
  extinguished = true;
  stop_all();

  // Servo 1
  Serial.println(F(">> SERVO 1 FIRING"));
  servo1.attach(SERVO1_PIN);
  delay(50);
  servo1.write(180);
  delay(750);

  // Release servo 1, then fire servo 2
  servo1.detach();
  delay(100);
  Serial.println(F(">> SERVO 2 FIRING"));
  servo2.attach(SERVO2_PIN);
  delay(50);
  servo2.write(180);

  Serial.println(F(">> BOTH EXTINGUISHERS DONE"));
}

void setup() {
  int pins[] = { FL_DIR1,FL_DIR2,FL_PWM, FR_DIR1,FR_DIR2,FR_PWM,
                 RL_DIR1,RL_DIR2,RL_PWM, RR_DIR1,RR_DIR2,RR_PWM };
  for (int i = 0; i < 12; i++) pinMode(pins[i], OUTPUT);
  stop_all();

  // Keep servo pins LOW — no twitch
  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);
  digitalWrite(SERVO1_PIN, LOW);
  digitalWrite(SERVO2_PIN, LOW);

  Serial.begin(9600);
  Serial.println(F("pbot v5 ready. F=fwd S=stop E=extinguish"));
  lastCmdTime = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    lastCmdTime = millis();

    if (c == 'F') { drive_forward(DRIVE_SPEED); }
    if (c == 'S') { stop_all(); }
    if (c == 'E') { fire_extinguishers(); }
  }

  if (moving && (millis() - lastCmdTime > WATCHDOG_MS)) {
    stop_all();
    Serial.println(F(">> WATCHDOG: auto-stop"));
  }
}
