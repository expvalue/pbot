#include <Arduino.h>
#line 1 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
/*
 * Arduino Mega fire robot firmware.
 *
 * Serial commands at 9600 baud:
 *   FIRE_AHEAD -> drive forward fast toward the fire
 *   FIRE_LEFT  -> turn left in place to center the fire
 *   FIRE_RIGHT -> turn right in place to center the fire
 *   FIRE_STOP  -> full stop
 *   FIRE_ON    -> start sweeping the extinguisher servo
 *   FIRE_OFF   -> stop the servo and return to center
 *
 * Servo wiring:
 *   Signal -> D44
 *   VCC    -> 5V
 *   GND    -> GND
 *
 * Motor wiring kept from the previous Mega setup.
 */

#include <Servo.h>

#define speedPinR 9
#define RightMotorDirPin1 22
#define RightMotorDirPin2 24

#define speedPinL 10
#define LeftMotorDirPin1 26
#define LeftMotorDirPin2 28

#define speedPinRB 11
#define RightMotorDirPin1B 5
#define RightMotorDirPin2B 6

#define speedPinLB 12
#define LeftMotorDirPin1B 7
#define LeftMotorDirPin2B 8

#define DRIVE_SPEED_FAST 220
#define TURN_SPEED_FAST 210

#define SERVO_PIN 44
#define SERVO_REST_ANGLE 90
#define SERVO_MIN_ANGLE 40
#define SERVO_MAX_ANGLE 140
#define SERVO_STEP_DEGREES 3
#define SERVO_UPDATE_MS 20

#define MOTOR_COMMAND_TIMEOUT_MS 500

Servo fireServo;
bool fireActive = false;
bool motorsMoving = false;
int servoAngle = SERVO_REST_ANGLE;
int servoDirection = 1;
unsigned long lastServoUpdateMs = 0;
unsigned long lastMotorCommandMs = 0;

#line 58 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void FR_fwd(int s);
#line 59 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void FL_fwd(int s);
#line 60 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void RR_fwd(int s);
#line 61 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void RL_fwd(int s);
#line 63 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void FR_rev(int s);
#line 64 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void FL_rev(int s);
#line 65 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void RR_rev(int s);
#line 66 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void RL_rev(int s);
#line 68 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void refreshMotorWatchdog();
#line 73 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void drive_forward_fast();
#line 82 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void turn_left_fast();
#line 91 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void turn_right_fast();
#line 100 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void stop_motors();
#line 120 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void fire_on();
#line 127 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void fire_off();
#line 135 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void update_fire_servo();
#line 158 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void update_motor_failsafe();
#line 169 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void handle_command(String cmd);
#line 193 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void init_GPIO();
#line 209 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void setup();
#line 222 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void loop();
#line 58 "C:\\Users\\saigu\\Downloads\\pbot\\firebot\\firebot.ino"
void FR_fwd(int s) { digitalWrite(RightMotorDirPin1, LOW); digitalWrite(RightMotorDirPin2, HIGH); analogWrite(speedPinR, s); }
void FL_fwd(int s) { digitalWrite(LeftMotorDirPin1, LOW); digitalWrite(LeftMotorDirPin2, HIGH); analogWrite(speedPinL, s); }
void RR_fwd(int s) { digitalWrite(RightMotorDirPin1B, LOW); digitalWrite(RightMotorDirPin2B, HIGH); analogWrite(speedPinRB, s); }
void RL_fwd(int s) { digitalWrite(LeftMotorDirPin1B, LOW); digitalWrite(LeftMotorDirPin2B, HIGH); analogWrite(speedPinLB, s); }

void FR_rev(int s) { digitalWrite(RightMotorDirPin1, HIGH); digitalWrite(RightMotorDirPin2, LOW); analogWrite(speedPinR, s); }
void FL_rev(int s) { digitalWrite(LeftMotorDirPin1, HIGH); digitalWrite(LeftMotorDirPin2, LOW); analogWrite(speedPinL, s); }
void RR_rev(int s) { digitalWrite(RightMotorDirPin1B, HIGH); digitalWrite(RightMotorDirPin2B, LOW); analogWrite(speedPinRB, s); }
void RL_rev(int s) { digitalWrite(LeftMotorDirPin1B, HIGH); digitalWrite(LeftMotorDirPin2B, LOW); analogWrite(speedPinLB, s); }

void refreshMotorWatchdog() {
  motorsMoving = true;
  lastMotorCommandMs = millis();
}

void drive_forward_fast() {
  FL_fwd(DRIVE_SPEED_FAST);
  FR_fwd(DRIVE_SPEED_FAST);
  RL_fwd(DRIVE_SPEED_FAST);
  RR_fwd(DRIVE_SPEED_FAST);
  refreshMotorWatchdog();
  Serial.println(F("CMD: FIRE_AHEAD"));
}

void turn_left_fast() {
  FL_rev(TURN_SPEED_FAST);
  FR_fwd(TURN_SPEED_FAST);
  RL_rev(TURN_SPEED_FAST);
  RR_fwd(TURN_SPEED_FAST);
  refreshMotorWatchdog();
  Serial.println(F("CMD: FIRE_LEFT"));
}

void turn_right_fast() {
  FL_fwd(TURN_SPEED_FAST);
  FR_rev(TURN_SPEED_FAST);
  RL_fwd(TURN_SPEED_FAST);
  RR_rev(TURN_SPEED_FAST);
  refreshMotorWatchdog();
  Serial.println(F("CMD: FIRE_RIGHT"));
}

void stop_motors() {
  analogWrite(speedPinR, 0);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinRB, 0);
  analogWrite(speedPinLB, 0);

  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, LOW);
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B, LOW);
  digitalWrite(LeftMotorDirPin1B, LOW);
  digitalWrite(LeftMotorDirPin2B, LOW);

  motorsMoving = false;
  lastMotorCommandMs = millis();
  Serial.println(F("CMD: FIRE_STOP"));
}

void fire_on() {
  if (!fireActive) {
    fireActive = true;
    Serial.println(F("CMD: FIRE_ON"));
  }
}

void fire_off() {
  fireActive = false;
  servoAngle = SERVO_REST_ANGLE;
  servoDirection = 1;
  fireServo.write(servoAngle);
  Serial.println(F("CMD: FIRE_OFF"));
}

void update_fire_servo() {
  if (!fireActive) {
    return;
  }

  unsigned long now = millis();
  if (now - lastServoUpdateMs < SERVO_UPDATE_MS) {
    return;
  }
  lastServoUpdateMs = now;

  servoAngle += servoDirection * SERVO_STEP_DEGREES;
  if (servoAngle >= SERVO_MAX_ANGLE) {
    servoAngle = SERVO_MAX_ANGLE;
    servoDirection = -1;
  } else if (servoAngle <= SERVO_MIN_ANGLE) {
    servoAngle = SERVO_MIN_ANGLE;
    servoDirection = 1;
  }

  fireServo.write(servoAngle);
}

void update_motor_failsafe() {
  if (!motorsMoving) {
    return;
  }

  if (millis() - lastMotorCommandMs > MOTOR_COMMAND_TIMEOUT_MS) {
    stop_motors();
    Serial.println(F("FAILSAFE: motor timeout"));
  }
}

void handle_command(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "FIRE_AHEAD" || cmd == "F") {
    drive_forward_fast();
  } else if (cmd == "FIRE_LEFT") {
    turn_left_fast();
  } else if (cmd == "FIRE_RIGHT") {
    turn_right_fast();
  } else if (cmd == "FIRE_STOP" || cmd == "S") {
    stop_motors();
  } else if (cmd == "FIRE_ON") {
    fire_on();
  } else if (cmd == "FIRE_OFF") {
    fire_off();
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(cmd);
  }
}

void init_GPIO() {
  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT);
  pinMode(RightMotorDirPin2B, OUTPUT);
  pinMode(speedPinRB, OUTPUT);
  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT);
  pinMode(speedPinLB, OUTPUT);
  stop_motors();
}

void setup() {
  init_GPIO();
  Serial.begin(9600);
  Serial.setTimeout(20);
  fireServo.attach(SERVO_PIN);
  fireServo.write(SERVO_REST_ANGLE);
  lastMotorCommandMs = millis();
#if defined(USBCON)
  while (!Serial) { ; }
#endif
  Serial.println(F("Mega ready. FIRE_AHEAD/FIRE_LEFT/FIRE_RIGHT/FIRE_STOP/FIRE_ON/FIRE_OFF"));
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handle_command(cmd);
  }

  update_fire_servo();
  update_motor_failsafe();
}

