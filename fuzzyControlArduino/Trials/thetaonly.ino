#include <Wire.h>
#include <math.h>
#include "MPU6050_tockn.h"
#include <Fuzzy.h>

// Motor driver pins
#define LEFT_IN1 8
#define LEFT_IN2 7
#define LEFT_EN  9
#define RIGHT_IN1 10
#define RIGHT_IN2 11
#define RIGHT_EN 9  // Shared with LEFT_EN

// Constants
const int DELTA_T = 100; // ms

// Global variables
MPU6050 mpu6050(Wire);
unsigned long current_time = 0;
unsigned long previous_time = 0;

// Tuning Parameters
float theta_input_scale = 1.0;
float theta_input_bias = 0.0;
float output_mf_scale = 1.0;
float output_mf_bias = 0.0;

// Complementary filter
float filteredAngle = 0;
float alpha = 0.96; // Lower to reduce vibration (less gyro trust)

// Fuzzy logic
Fuzzy* fuzzy = new Fuzzy();
FuzzyInput* theta = new FuzzyInput(1);
FuzzyOutput* balance = new FuzzyOutput(1);

// Global fuzzy sets for debugging
FuzzySet* theta_neg;
FuzzySet* theta_bal;
FuzzySet* theta_pos;

// Motor Controller Class
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

MotorController leftMotor(LEFT_IN1, LEFT_IN2);
MotorController rightMotor(RIGHT_IN1, RIGHT_IN2);

void setup() {
  Serial.begin(115200);
  Serial.println("Self-Balancing Robot (Theta Only + Complementary Filter)");

  Wire.begin();
  mpu6050.begin();

  delay(100);
  Serial.println("Calibrating gyro...");
  delay(5000);
  mpu6050.calcGyroOffsets(true);
  Serial.println("Calibration done.");
  Serial.print("GyroX offset: "); Serial.println(mpu6050.getGyroXoffset());

  setupFuzzyLogic();
  pinMode(LEFT_EN, OUTPUT);
}

void loop() {
  current_time = millis();
  mpu6050.update();

  if ((current_time - previous_time) > DELTA_T) {
    float dt = (current_time - previous_time) / 1000.0; // Convert ms to seconds

    float acc_angle = mpu6050.getAccAngleX(); // From accelerometer
    float gyro_rate = mpu6050.getGyroX();     // Angular velocity (deg/s)

    // Complementary filter to fuse gyro + accel
    filteredAngle = alpha * (filteredAngle + gyro_rate * dt) + (1.0 - alpha) * acc_angle;

    float theta_val = filteredAngle;

    if (isnan(theta_val)) {
      Serial.println("Sensor error!");
      analogWrite(LEFT_EN, 0);
      return;
    }

    float scaled_theta = theta_val * theta_input_scale + theta_input_bias;

    fuzzy->setInput(1, scaled_theta);
    fuzzy->fuzzify();

    // Print which membership functions are activated
   
    Serial.print("Theta (filtered): "); Serial.println(theta_val);
    


    float output = fuzzy->defuzzify(1);
    float balance_voltage = output * output_mf_scale + output_mf_bias;
    balance_voltage = constrain(balance_voltage, -9.0, 9.0); // Allow for more torque range

    float pwm_command = (balance_voltage * 255.0) / 3; // Scale for ±9V
    pwm_command = constrain(pwm_command, -255.0, 255.0); // Clamp to max

    leftMotor.setDirection((int)pwm_command);
    rightMotor.setDirection((int)pwm_command);
    analogWrite(LEFT_EN, abs((int)pwm_command));

    Serial.print("Output Voltage: "); Serial.println(balance_voltage);
    Serial.print("PWM: "); Serial.println(pwm_command);
    Serial.println();

    previous_time = current_time;
  }
}

void setupFuzzyLogic() {
  // Input: theta angle
  FuzzySet* theta_neg_hi = new FuzzySet(-60, -45, -40, -30);
  FuzzySet* theta_neg    = new FuzzySet(-40, -25, -15, -3);
  theta_bal              = new FuzzySet(-3, 0, 0, 3);  // still global
  FuzzySet* theta_pos    = new FuzzySet(3, 15, 25, 40);
  FuzzySet* theta_pos_hi = new FuzzySet(30, 40, 45, 60);

  theta->addFuzzySet(theta_neg_hi);
  theta->addFuzzySet(theta_neg);
  theta->addFuzzySet(theta_bal);
  theta->addFuzzySet(theta_pos);
  theta->addFuzzySet(theta_pos_hi);
  fuzzy->addFuzzyInput(theta);

  // Output: balance voltage
  FuzzySet* out_high_neg = new FuzzySet(-10, -9, -8, -6);
  FuzzySet* out_low_neg  = new FuzzySet(-6, -4, -3, -1.5);
  FuzzySet* out_zero     = new FuzzySet(-1.5, 0, 0, 1.5);
  FuzzySet* out_low_pos  = new FuzzySet(1.5, 3, 4, 6);
  FuzzySet* out_high_pos = new FuzzySet(6, 8, 9, 10);

  balance->addFuzzySet(out_high_neg);
  balance->addFuzzySet(out_low_neg);
  balance->addFuzzySet(out_zero);
  balance->addFuzzySet(out_low_pos);
  balance->addFuzzySet(out_high_pos);
  fuzzy->addFuzzyOutput(balance);

  // Rules
  FuzzyRuleAntecedent* if_theta_neg_hi = new FuzzyRuleAntecedent();
  if_theta_neg_hi->joinSingle(theta_neg_hi);
  FuzzyRuleConsequent* then_output_high_neg = new FuzzyRuleConsequent();
  then_output_high_neg->addOutput(out_high_neg);
  fuzzy->addFuzzyRule(new FuzzyRule(1, if_theta_neg_hi, then_output_high_neg));

  FuzzyRuleAntecedent* if_theta_neg = new FuzzyRuleAntecedent();
  if_theta_neg->joinSingle(theta_neg);
  FuzzyRuleConsequent* then_output_low_neg = new FuzzyRuleConsequent();
  then_output_low_neg->addOutput(out_low_neg);
  fuzzy->addFuzzyRule(new FuzzyRule(2, if_theta_neg, then_output_low_neg));

  FuzzyRuleAntecedent* if_theta_bal = new FuzzyRuleAntecedent();
  if_theta_bal->joinSingle(theta_bal);
  FuzzyRuleConsequent* then_output_zero = new FuzzyRuleConsequent();
  then_output_zero->addOutput(out_zero);
  fuzzy->addFuzzyRule(new FuzzyRule(3, if_theta_bal, then_output_zero));

  FuzzyRuleAntecedent* if_theta_pos = new FuzzyRuleAntecedent();
  if_theta_pos->joinSingle(theta_pos);
  FuzzyRuleConsequent* then_output_low_pos = new FuzzyRuleConsequent();
  then_output_low_pos->addOutput(out_low_pos);
  fuzzy->addFuzzyRule(new FuzzyRule(4, if_theta_pos, then_output_low_pos));

  FuzzyRuleAntecedent* if_theta_pos_hi = new FuzzyRuleAntecedent();
  if_theta_pos_hi->joinSingle(theta_pos_hi);
  FuzzyRuleConsequent* then_output_high_pos = new FuzzyRuleConsequent();
  then_output_high_pos->addOutput(out_high_pos);
  fuzzy->addFuzzyRule(new FuzzyRule(5, if_theta_pos_hi, then_output_high_pos));}