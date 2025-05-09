#include <Wire.h>
#include <math.h>
#include "MPU6050_tockn.h"

// Motor driver pins
#define LEFT_IN1 9
#define LEFT_IN2 8
#define LEFT_EN  10

#define RIGHT_IN1 11
#define RIGHT_IN2 12
#define RIGHT_EN 5

// Encoder pins (optional)
#define leftmotor_A 2
#define leftmotor_B 4

// Constants
const float SETPOINT = 0.0;     // Target angle (upright)
const int DELTA_T = 10;         // Loop time in ms
const float MAX_PWM = 255.0;    // Max motor signal

// PID Gains (adjust based on behavior)
float Kp = 15.0;
float Ki = 0.3;
float Kd = 2.0;

// Variables
MPU6050 mpu(Wire);
volatile int left_count = 0;
float error_sum = 0;
float last_error = 0;
unsigned long last_time = 0;

// ========== Motor Control Class ==========
class MotorController {
  private:
    int in1, in2, en;
  public:
    MotorController(int a, int b, int e) : in1(a), in2(b), en(e) {
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(en, OUTPUT);
    }

    void setPWM(float pwm) {
      pwm = -pwm;  // flip if needed — test your setup!
      pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

      if (pwm > 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
      } else if (pwm < 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
      } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
      }

      analogWrite(en, constrain(abs((int)pwm), 0, 255));
    }
};

MotorController leftMotor(LEFT_IN1, LEFT_IN2, LEFT_EN);
MotorController rightMotor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN);

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();

  delay(500);
  Serial.println("Calibrating MPU6050...");
  delay(3000);
  mpu.calcGyroOffsets(true);
  Serial.println("Calibration complete.");

  pinMode(leftmotor_A, INPUT);
  pinMode(leftmotor_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftmotor_A), phase_A_left, RISING);

  last_time = millis();
}

// ========== Main Loop ==========
void loop() {
  unsigned long now = millis();
  if (now - last_time >= DELTA_T) {
    mpu.update();

    float angle = mpu.getAngleX();
    float gyro = mpu.getGyroX();

    float error = SETPOINT - angle;
    error_sum += error * DELTA_T;
    error_sum = constrain(error_sum, -1000, 1000);  // Prevent wind-up

    float d_error = (error - last_error) / DELTA_T;
    last_error = error;

    float P = Kp * error;
    float I = Ki * error_sum;
    float D = Kd * d_error;
    float output = P + I + D;
    output = constrain(output, -MAX_PWM, MAX_PWM);

    leftMotor.setPWM(output);
    rightMotor.setPWM(output);

    // ======= Serial Logging =======
    Serial.print("Angle: "); Serial.print(angle, 2);
    Serial.print(" | P: "); Serial.print(P, 2);
    Serial.print(" | I: "); Serial.print(I, 2);
    Serial.print(" | D: "); Serial.print(D, 2);
    Serial.print(" | OUT: "); Serial.println(output, 2);

    last_time = now;
  }
}

// ========== Encoder Interrupt ==========
void phase_A_left() {
  if (digitalRead(leftmotor_A)) {
    left_count += digitalRead(leftmotor_B) ? -1 : 1;
  } else {
    left_count += digitalRead(leftmotor_B) ? 1 : -1;
  }
}
