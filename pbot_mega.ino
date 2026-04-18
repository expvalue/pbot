/*
 * pbot_mega.ino — Production firmware for fire-detection bot
 * Arduino Mega  •  9600 baud  •  Osoyoo Mecanum
 *
 * Accepts serial commands from firebot.py:
 *   'F' → drive forward (all 4 wheels)
 *   'S' → stop all motors
 */

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

#define DRIVE_SPEED 150

void FL_fwd(int s) { digitalWrite(FL_DIR1, HIGH); digitalWrite(FL_DIR2, LOW);  analogWrite(FL_PWM, s); }
void FR_fwd(int s) { digitalWrite(FR_DIR1, HIGH); digitalWrite(FR_DIR2, LOW);  analogWrite(FR_PWM, s); }
void RL_fwd(int s) { digitalWrite(RL_DIR1, HIGH); digitalWrite(RL_DIR2, LOW);  analogWrite(RL_PWM, s); }
void RR_fwd(int s) { digitalWrite(RR_DIR1, HIGH); digitalWrite(RR_DIR2, LOW);  analogWrite(RR_PWM, s); }

void drive_forward(int s) {
  FL_fwd(s); FR_fwd(s); RL_fwd(s); RR_fwd(s);
  Serial.println(F(">> FORWARD"));
}

void stop_all() {
  analogWrite(FL_PWM, 0);
  analogWrite(FR_PWM, 0);
  analogWrite(RL_PWM, 0);
  analogWrite(RR_PWM, 0);
  Serial.println(F(">> STOP"));
}

void setup() {
  int pins[] = { FL_DIR1,FL_DIR2,FL_PWM, FR_DIR1,FR_DIR2,FR_PWM,
                 RL_DIR1,RL_DIR2,RL_PWM, RR_DIR1,RR_DIR2,RR_PWM };
  for (int i = 0; i < 12; i++) pinMode(pins[i], OUTPUT);
  stop_all();
  Serial.begin(9600);
  Serial.println(F("pbot ready. Waiting for commands..."));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if      (c == 'F') drive_forward(DRIVE_SPEED);
    else if (c == 'S') stop_all();
  }
}
