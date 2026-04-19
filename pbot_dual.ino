/*
 * pbot_dual.ino — Fire bot with staggered extinguisher servos
 * Arduino Mega  •  9600 baud  •  Osoyoo Mecanum
 *
 * Commands from firebot_dual.py:
 *   'F' → drive forward (all 4 wheels)
 *   'S' → stop all motors
 *   '1' → trigger servo 1 only (extinguisher 1)
 *   '2' → trigger servo 2 only (extinguisher 2)
 *
 * Servos:
 *   D3 → extinguisher 1 (triggered at 1.3s)
 *   D4 → extinguisher 2 (triggered at 2.0s)
 *
 * SAFETY: auto-stops if no command received for 1 second
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

unsigned long lastCmdTime = 0;
bool          moving      = false;
bool          servo1_fired = false;
bool          servo2_fired = false;

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

void setup() {
  int pins[] = { FL_DIR1,FL_DIR2,FL_PWM, FR_DIR1,FR_DIR2,FR_PWM,
                 RL_DIR1,RL_DIR2,RL_PWM, RR_DIR1,RR_DIR2,RR_PWM };
  for (int i = 0; i < 12; i++) pinMode(pins[i], OUTPUT);
  stop_all();

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(0);
  servo2.write(0);

  Serial.begin(9600);
  Serial.println(F("pbot ready. F=fwd S=stop 1=servo1 2=servo2"));
  lastCmdTime = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'F') { drive_forward(DRIVE_SPEED); lastCmdTime = millis(); }
    if (c == 'S') { stop_all();                 lastCmdTime = millis(); }
    if (c == '1' && !servo1_fired) {
      stop_all();
      servo1.write(180);
      servo1_fired = true;
      Serial.println(F(">> SERVO 1 FIRED"));
      lastCmdTime = millis();
    }
    if (c == '2' && !servo2_fired) {
      stop_all();
      servo2.write(180);
      servo2_fired = true;
      Serial.println(F(">> SERVO 2 FIRED"));
      lastCmdTime = millis();
    }
  }

  if (moving && (millis() - lastCmdTime > WATCHDOG_MS)) {
    stop_all();
    Serial.println(F(">> WATCHDOG: auto-stop"));
  }
}
