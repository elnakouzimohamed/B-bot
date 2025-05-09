#define LEFT_IN1 7
#define LEFT_IN2 8
#define LEFT_EN  9

#define RIGHT_IN2 11
#define RIGHT_IN1 10
#define RIGHT_EN 9

void setup() {
  // Set motor pins as outputs
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_EN, OUTPUT);

  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);


  // Set both motors to run forward
 digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(RIGHT_IN1, HIGH);
  digitalWrite(RIGHT_IN2, LOW);

  // Full speed (255)

 // analogWrite(LEFT_EN, 255);
 //pinMode(9, OUTPUT);
  analogWrite(9, 255);
  //analogWrite(9, 255);
 
}

void loop() {
  // Nothing here — motors continue running forward at full speed
}
