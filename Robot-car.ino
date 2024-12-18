// Pin Definitions
const int LEFT_MOTOR_FORWARD = 9; // Pin for driving the left motor forward
const int LEFT_MOTOR_REVERSE = 8; // Pin for driving the left motor in reverse
const int RIGHT_MOTOR_FORWARD = 7; // Pin for driving the right motor forward
const int RIGHT_MOTOR_REVERSE = 6; // Pin for driving the right motor in reverse
const int LEFT_MOTOR_PWM = 5; // PWM pin for left motor speed control
const int RIGHT_MOTOR_PWM = 10; // PWM pin for right motor speed control

const int ULTRASONIC_LEFT_TRIG = 2; // Trigger pin for the left ultrasonic sensor
const int ULTRASONIC_LEFT_ECHO = 3; // Echo pin for the left ultrasonic sensor
const int ULTRASONIC_CENTER_TRIG = 4; // Trigger pin for the center ultrasonic sensor
const int ULTRASONIC_CENTER_ECHO = 11; // Echo pin for the center ultrasonic sensor
const int ULTRASONIC_RIGHT_TRIG = 12; // Trigger pin for the right ultrasonic sensor
const int ULTRASONIC_RIGHT_ECHO = 13; // Echo pin for the right ultrasonic sensor

// Constants
const int MAX_DISTANCE = 1000; // Maximum measurable distance in mm
const int MIN_DISTANCE = 200;  // Minimum safe distance in mm
const int SPEED_OF_SOUND = 343; // Speed of sound in m/s
const int PWM_MIN = 120;       // Minimum PWM speed
const int PWM_MAX = 255;       // Maximum PWM speed
const int SCAN_INTERVAL = 100; // Time between sensor scans in ms
const int BACKUP_DELAY = 200;  // Delay for reverse maneuver in ms
const int SPIN_DELAY = 800;    // Delay for spin maneuver in ms

// Variables
int leftMotorSpeed = 0; // Current speed of the left motor
int rightMotorSpeed = 0; // Current speed of the right motor
int pwmAdjustment = 0; // Adjustment for speed difference between motors
int scanDistances[3] = {0, 0, 0}; // Distances from left, center, and right sensors
unsigned long lastScanTime = 0; // Time of the last sensor scan
unsigned long speedUpdateTime = 0; // Time of the last speed adjustment
bool isReversing = false; // Flag to indicate if the car is in a reversing maneuver

void setup() {
  Serial.begin(9600);
  Serial.println("EnhancedRobotCar Initialized");

  // Motor pin setup
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_REVERSE, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_REVERSE, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  // Ultrasonic sensor pin setup
  pinMode(ULTRASONIC_LEFT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_LEFT_ECHO, INPUT);
  pinMode(ULTRASONIC_CENTER_TRIG, OUTPUT);
  pinMode(ULTRASONIC_CENTER_ECHO, INPUT);
  pinMode(ULTRASONIC_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_RIGHT_ECHO, INPUT);

  delay(3000); // Startup delay
}

void loop() {
  performSensorScans(); // Scan the environment using ultrasonic sensors

  if (millis() > speedUpdateTime) {
    adjustSpeedAndDirection(); // Update motor speed and direction
    speedUpdateTime = millis() + 10;
  }

  logDiagnostics(); // Log diagnostic information to Serial Monitor
}

// Perform scans with the ultrasonic sensors
// This function updates the scanDistances array with the current measurements from each sensor
void performSensorScans() {
  if (millis() > lastScanTime) {
    scanDistances[0] = measureDistance(ULTRASONIC_LEFT_TRIG, ULTRASONIC_LEFT_ECHO); // Left sensor
    scanDistances[1] = measureDistance(ULTRASONIC_CENTER_TRIG, ULTRASONIC_CENTER_ECHO); // Center sensor
    scanDistances[2] = measureDistance(ULTRASONIC_RIGHT_TRIG, ULTRASONIC_RIGHT_ECHO); // Right sensor

    lastScanTime = millis() + SCAN_INTERVAL;
  }
}

// Measure distance using an ultrasonic sensor
// Sends a pulse and calculates the distance based on the echo time
int measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); // Ensure trigger pin is LOW
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); // Send a 10-microsecond pulse
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH); // Measure pulse duration
  int distance = (duration * SPEED_OF_SOUND / 2000); // Convert to distance in mm

  return constrain(distance, 0, MAX_DISTANCE); // Return constrained distance
}

// Adjust motor speed and direction based on sensor data
// Calculates appropriate speeds and handles obstacle avoidance logic
void adjustSpeedAndDirection() {
  int forwardSpeed = constrain(scanDistances[1] - 255, 0, PWM_MAX); // Adjust speed based on center sensor

  // If too close to obstacles on all sides, perform recovery
  if (scanDistances[0] < MIN_DISTANCE && scanDistances[1] < MIN_DISTANCE && scanDistances[2] < MIN_DISTANCE) {
    performRecoveryManeuver();
    return;
  }

  // Adjust direction based on left and right sensor readings
  if (scanDistances[0] < 500 || scanDistances[2] < 500) {
    pwmAdjustment += (scanDistances[0] > scanDistances[2]) ? 1 : -1;
  } else {
    pwmAdjustment = 0;
  }

  pwmAdjustment = constrain(pwmAdjustment, -200, 200); // Constrain adjustment value

  leftMotorSpeed = constrain(forwardSpeed + pwmAdjustment, PWM_MIN, PWM_MAX); // Calculate left motor speed
  rightMotorSpeed = constrain(forwardSpeed - pwmAdjustment, PWM_MIN, PWM_MAX); // Calculate right motor speed

  driveMotors(leftMotorSpeed, rightMotorSpeed); // Drive the motors
}

// Control the motors
// Sets the motor pins to drive the car forward or in reverse
void driveMotors(int leftSpeed, int rightSpeed) {
  if (!isReversing) {
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(LEFT_MOTOR_REVERSE, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_REVERSE, LOW);

    analogWrite(LEFT_MOTOR_PWM, leftSpeed); // Set left motor speed
    analogWrite(RIGHT_MOTOR_PWM, rightSpeed); // Set right motor speed
  }
}

// Perform recovery maneuver when stuck
// Reverses and spins the car to free it from obstacles
void performRecoveryManeuver() {
  isReversing = true;

  // Reverse both motors
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_REVERSE, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_REVERSE, HIGH);

  analogWrite(LEFT_MOTOR_PWM, 200);
  analogWrite(RIGHT_MOTOR_PWM, 200);
  delay(BACKUP_DELAY);

  // Spin the car
  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(LEFT_MOTOR_REVERSE, LOW);
  delay(SPIN_DELAY);

  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);

  isReversing = false;
  speedUpdateTime = millis() + 1000; // Delay before resuming
}

// Log diagnostic information
// Outputs sensor readings and motor speeds to the Serial Monitor
void logDiagnostics() {
  // Logs diagnostic information to the Serial Monitor for debugging purposes
  Serial.print("Left: ");
  Serial.print(scanDistances[0]);
  Serial.print(" mm, Center: ");
  Serial.print(scanDistances[1]);
  Serial.print(" mm, Right: ");
  Serial.println(scanDistances[2]);

  Serial.print("Left Motor: ");
  Serial.print(leftMotorSpeed);
  Serial.print(", Right Motor: ");
  Serial.println(rightMotorSpeed);
}
