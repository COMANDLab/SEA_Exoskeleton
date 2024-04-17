#include <Arduino.h>
#include <util/atomic.h>

#define ENCODER_A 2 // 5th gray wire of Motor
#define ENCODER_B 3 // 2nd gray wire of Motor
#define PWM 9 //PWM on Motor driver
#define AI1 8 // AI1 on Motor driver
#define AI2 7 // AI2 on Motor driver

void readEncoder();
void setMotor(int dir, int speed, int pwm, int in1, int in2);

volatile int position = 0;
volatile int target =1250;
long prevT = 0;
float eprev = 0;
float eint = 0;

void setup() {

Serial.begin(9600);
pinMode(ENCODER_A, INPUT);
pinMode(ENCODER_B, INPUT);
attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, RISING);

pinMode(PWM, OUTPUT);
pinMode(AI1, OUTPUT);
pinMode(AI2, OUTPUT);
}

void loop() {

  // PID Constants
  float Kp = 8;
  float Kd = 0.05;
  float Ki = 0.0;

  long currT = millis();
  float dt = float(currT - prevT) / 1000.0;
  prevT = currT;

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = position;
  }

  int e = pos - target;
  float dedt = (e - eprev) / dt;
  eint += e * dt;

  float u = Kp * e + Kd * dedt + Ki * eint;

  float pwr =fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  setMotor(dir, pwr, PWM, AI1, AI2);

  eprev = e;

  Serial.print("Pos: ");
  Serial.println(pos);
  Serial.print("Target: ");
  Serial.println(target);
  Serial.flush();
}

void setMotor(int dir, int speed, int pwm, int in1, int in2) {
  analogWrite(pwm, speed);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm, speed);
}

void readEncoder() {
  int b = digitalRead(ENCODER_B);
  if (b > 0) {
    position++;
  } else {
    position--;
  }
}