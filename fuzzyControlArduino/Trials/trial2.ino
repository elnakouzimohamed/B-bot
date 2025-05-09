#include <Wire.h>
#include <math.h>
#include "MPU6050_tockn.h"
#include <Fuzzy.h>

// Fuzzy Input Scaling
float theta_input_scale = 1.0;
float theta_input_bias = 0.0;

float dtheta_input_scale = 0.75;
float dtheta_input_bias = 0.0;

// Fuzzy Output Scaling
float output_mf_scale = 1.0;
float output_mf_bias = 0.0;

// Motor driver pins
#define LEFT_IN2 8
#define LEFT_IN1 7
#define LEFT_EN  9
#define RIGHT_IN1 11
#define RIGHT_IN2 10
#define RIGHT_EN 9  // Same as LEFT_EN

// Constants
const int DELTA_T = 5; // ms

// Global variables
MPU6050 mpu6050(Wire);
unsigned long current_time = 0;
unsigned long previous_time = 0;

// Complementary filter
float alpha = 0.96;
float filteredAngle = 0.0;
float filteredDTheta = 0.0;

// Fuzzy logic components
Fuzzy* fuzzy = new Fuzzy();
FuzzyInput* theta = new FuzzyInput(1);
FuzzyInput* dtheta = new FuzzyInput(2);
FuzzyOutput* balance = new FuzzyOutput(1);

// Motor Controller Class (Direction Only)
class MotorController {
  private:
    int in1Pin;
    int in2Pin;

  public:
    MotorController(int in1, int in2) {
      in1Pin = in1;
      in2Pin = in2;
      pinMode(in1Pin, OUTPUT);
      pinMode(in2Pin, OUTPUT);
    }

    void setDirection(int pwm) {
      if (pwm > 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
      } else if (pwm < 0) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
      } else {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
      }
    }
};

// Motors (no need for EN in the controller)
MotorController leftMotor(LEFT_IN1, LEFT_IN2);
MotorController rightMotor(RIGHT_IN1, RIGHT_IN2);

void setup() {
  Serial.begin(115200);
  Serial.println("Self-Balancing Robot Starting...");

  Wire.begin();
  mpu6050.begin();
  Serial.println("MPU6050 Initialized.");

  delay(100);
  Serial.println("Please keep the robot still for gyro calibration...");
  delay(5000);
  mpu6050.calcGyroOffsets(true);
  Serial.println("Gyroscope Calibration Completed.");

  setupFuzzyLogic();
  pinMode(LEFT_EN, OUTPUT);  // PWM shared
  Serial.println("Setup Completed.");
}

void loop() {
  current_time = millis();
  mpu6050.update();

  if ((current_time - previous_time) > DELTA_T) {
    float dt = (current_time - previous_time) / 1000.0;

    float accAngle = mpu6050.getAccAngleX();
    float gyroRate = mpu6050.getGyroX();

    filteredAngle = alpha * (filteredAngle + gyroRate * dt) + (1 - alpha) * accAngle;

    // Smoothed dTheta
    filteredDTheta = 0.9 * filteredDTheta + 0.1 * gyroRate;

    if (isnan(filteredAngle) || isnan(filteredDTheta)) {
      Serial.println("Sensor error! Stopping motors.");
      analogWrite(LEFT_EN, 0);
      return;
    }

    float balance_voltage = computeBalanceVoltage(filteredAngle, filteredDTheta);
    balance_voltage = constrain(balance_voltage, -2.5, 2.5);

    float pwm_command = (balance_voltage * 255.0) / 3.0;
    pwm_command = constrain(pwm_command, -255.0, 255.0);

    // Dead zone
    if (abs(pwm_command) < 20) pwm_command = 0;

    leftMotor.setDirection((int)pwm_command);
    rightMotor.setDirection((int)pwm_command);

    analogWrite(LEFT_EN, min(abs((int)(pwm_command * 0.5)), 255));

    Serial.println("========== Robot State ==========");
    Serial.print("Theta Error (filtered): "); Serial.print(filteredAngle); Serial.println(" deg");
    Serial.print("dTheta (filtered): "); Serial.print(filteredDTheta); Serial.println(" deg/s");
    Serial.print("Fuzzy Output (Voltage): "); Serial.println(balance_voltage);
    Serial.print("PWM Command: "); Serial.println(pwm_command);
    Serial.println("==================================");

    previous_time = current_time;
  }
}
void setupFuzzyLogic() {
  // Theta input sets
  FuzzySet* theta_neg = new FuzzySet(
    theta_input_scale * (-45 + theta_input_bias),
    theta_input_scale * (-20 + theta_input_bias),
    theta_input_scale * (-20 + theta_input_bias),
    theta_input_scale * (-5  + theta_input_bias));

  FuzzySet* theta_bal = new FuzzySet(
    theta_input_scale * (-5 + theta_input_bias),
    theta_input_scale * (  0 + theta_input_bias),
    theta_input_scale * (  0 + theta_input_bias),
    theta_input_scale * ( 5 + theta_input_bias));

  FuzzySet* theta_pos = new FuzzySet(
    theta_input_scale * (  5 + theta_input_bias),
    theta_input_scale * ( 20 + theta_input_bias),
    theta_input_scale * ( 20 + theta_input_bias),
    theta_input_scale * ( 45 + theta_input_bias));

  theta->addFuzzySet(theta_neg);
  theta->addFuzzySet(theta_bal);
  theta->addFuzzySet(theta_pos);
  fuzzy->addFuzzyInput(theta);

  // dTheta input sets
  FuzzySet* dtheta_neg = new FuzzySet(
    dtheta_input_scale * (-60 + dtheta_input_bias),
    dtheta_input_scale * (-30 + dtheta_input_bias),
    dtheta_input_scale * (-30 + dtheta_input_bias),
    dtheta_input_scale * ( -5 + dtheta_input_bias));

  FuzzySet* dtheta_zero = new FuzzySet(
    dtheta_input_scale * (-5 + dtheta_input_bias),
    dtheta_input_scale * (   0 + dtheta_input_bias),
    dtheta_input_scale * (   0 + dtheta_input_bias),
    dtheta_input_scale * ( 5 + dtheta_input_bias));

  FuzzySet* dtheta_pos = new FuzzySet(
    dtheta_input_scale * (  5 + dtheta_input_bias),
    dtheta_input_scale * ( 30 + dtheta_input_bias),
    dtheta_input_scale * ( 30 + dtheta_input_bias),
    dtheta_input_scale * ( 60 + dtheta_input_bias));

  dtheta->addFuzzySet(dtheta_neg);
  dtheta->addFuzzySet(dtheta_zero);
  dtheta->addFuzzySet(dtheta_pos);
  fuzzy->addFuzzyInput(dtheta);

  // Balance output sets
  FuzzySet* out_high_neg = new FuzzySet(
    output_mf_scale * (-5.0 + output_mf_bias),
    output_mf_scale * (-3.0 + output_mf_bias),
    output_mf_scale * (-2.0 + output_mf_bias),
    output_mf_scale * (-1.0 + output_mf_bias));

  FuzzySet* out_low_neg = new FuzzySet(
    output_mf_scale * (-2.0 + output_mf_bias),
    output_mf_scale * (-1.0 + output_mf_bias),
    output_mf_scale * (-0.5 + output_mf_bias),
    output_mf_scale * ( 0.0 + output_mf_bias));

  FuzzySet* out_average = new FuzzySet(
    output_mf_scale * (-0.25 + output_mf_bias),
    output_mf_scale * ( 0.0   + output_mf_bias),
    output_mf_scale * ( 0.0   + output_mf_bias),
    output_mf_scale * ( 0.25  + output_mf_bias));

  FuzzySet* out_low_pos = new FuzzySet(
    output_mf_scale * ( 0.0 + output_mf_bias),
    output_mf_scale * ( 0.5 + output_mf_bias),
    output_mf_scale * ( 1.0 + output_mf_bias),
    output_mf_scale * ( 2.0 + output_mf_bias));

  FuzzySet* out_high_pos = new FuzzySet(
    output_mf_scale * ( 1.0 + output_mf_bias),
    output_mf_scale * ( 2.0 + output_mf_bias),
    output_mf_scale * ( 3.0 + output_mf_bias),
    output_mf_scale * ( 5.0 + output_mf_bias));

  balance->addFuzzySet(out_high_neg);
  balance->addFuzzySet(out_low_neg);
  balance->addFuzzySet(out_average);
  balance->addFuzzySet(out_low_pos);
  balance->addFuzzySet(out_high_pos);
  fuzzy->addFuzzyOutput(balance);

  // === Modified Fuzzy Rules ===
  // For theta_pos → always strong backward (out_high_neg)
  FuzzyRuleAntecedent* r1 = new FuzzyRuleAntecedent();
  r1->joinWithAND(theta_pos, dtheta_pos);
  FuzzyRuleConsequent* c1 = new FuzzyRuleConsequent();
  c1->addOutput(out_high_neg);
  fuzzy->addFuzzyRule(new FuzzyRule(1, r1, c1));

  FuzzyRuleAntecedent* r2 = new FuzzyRuleAntecedent();
  r2->joinWithAND(theta_pos, dtheta_zero);
  FuzzyRuleConsequent* c2 = new FuzzyRuleConsequent();
  c2->addOutput(out_high_neg);
  fuzzy->addFuzzyRule(new FuzzyRule(2, r2, c2));

  FuzzyRuleAntecedent* r3 = new FuzzyRuleAntecedent();
  r3->joinWithAND(theta_pos, dtheta_neg);
  FuzzyRuleConsequent* c3 = new FuzzyRuleConsequent();
  c3->addOutput(out_high_neg);
  fuzzy->addFuzzyRule(new FuzzyRule(3, r3, c3));

  // For theta_neg → always strong forward (out_high_pos)
  FuzzyRuleAntecedent* r4 = new FuzzyRuleAntecedent();
  r4->joinWithAND(theta_neg, dtheta_pos);
  FuzzyRuleConsequent* c4 = new FuzzyRuleConsequent();
  c4->addOutput(out_high_pos);
  fuzzy->addFuzzyRule(new FuzzyRule(4, r4, c4));

  FuzzyRuleAntecedent* r5 = new FuzzyRuleAntecedent();
  r5->joinWithAND(theta_neg, dtheta_zero);
  FuzzyRuleConsequent* c5 = new FuzzyRuleConsequent();
  c5->addOutput(out_high_pos);
  fuzzy->addFuzzyRule(new FuzzyRule(5, r5, c5));

  FuzzyRuleAntecedent* r6 = new FuzzyRuleAntecedent();
  r6->joinWithAND(theta_neg, dtheta_neg);
  FuzzyRuleConsequent* c6 = new FuzzyRuleConsequent();
  c6->addOutput(out_high_pos);
  fuzzy->addFuzzyRule(new FuzzyRule(6, r6, c6));

  // For balanced angle → different response depending on dtheta
  FuzzyRuleAntecedent* r7 = new FuzzyRuleAntecedent();
  r7->joinWithAND(theta_bal, dtheta_neg);
  FuzzyRuleConsequent* c7 = new FuzzyRuleConsequent();
  c7->addOutput(out_low_neg);
  fuzzy->addFuzzyRule(new FuzzyRule(7, r7, c7));

  FuzzyRuleAntecedent* r8 = new FuzzyRuleAntecedent();
  r8->joinWithAND(theta_bal, dtheta_zero);
  FuzzyRuleConsequent* c8 = new FuzzyRuleConsequent();
  c8->addOutput(out_average);
  fuzzy->addFuzzyRule(new FuzzyRule(8, r8, c8));

  FuzzyRuleAntecedent* r9 = new FuzzyRuleAntecedent();
  r9->joinWithAND(theta_bal, dtheta_pos);
  FuzzyRuleConsequent* c9 = new FuzzyRuleConsequent();
  c9->addOutput(out_low_pos);
  fuzzy->addFuzzyRule(new FuzzyRule(9, r9, c9));
}


float computeBalanceVoltage(float theta_val, float dtheta_val) {
  fuzzy->setInput(1, theta_val);
  fuzzy->setInput(2, dtheta_val);
  fuzzy->fuzzify();
  return fuzzy->defuzzify(1);
}