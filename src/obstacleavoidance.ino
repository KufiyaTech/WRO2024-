#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <Pixy2.h>

// Pin Definitions
#define SERVO_PIN 9
#define INITIAL_ANGLE 90
#define IN1 11
#define IN2 12
#define ENA 13
#define trigPinl 3
#define echoPinl 6
#define trigPinr 2
#define echoPinr 5
#define trigPinf 4
#define echoPinf 7

MPU6050 mpu;
Servo myServo;
Pixy2 pixy;

// Variables
int16_t gz;
long previous_time;
float yaw_angle = 0;
float gyro_z_offset = 0;
int calibration_count = 2000;  // Number of samples for calibration
int currentAngle = INITIAL_ANGLE;
unsigned long lastTime = 0;
int targetYawAngle = 0;

// PID parameters
float Kp = 1;
float Ki = 0;
float Kd = 0;
// PID variables
float error;
float previousError = 0;
float integral = 0;
float derivative;
float controlSignal;
unsigned long previousTime;
unsigned long currentTime;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pixy.init();
  mpu.initialize();
  
  // Motor Driver setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  
  // Ultrasonic sensor setup
  pinMode(trigPinl, OUTPUT);
  pinMode(echoPinl, INPUT);
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);
  pinMode(trigPinf, OUTPUT);
  pinMode(echoPinf, INPUT);
  
  // Check if MPU6050 is connected properly
  Serial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");
  
  // Calibrate the gyro to calculate drift
  Serial.println("Calibrating gyro...");
  calculateGyroDrift();
  Serial.println("Done Calibrating");
  
  // Servo setup
  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);

  digitalWrite(IN1, HIGH);  // Set direction
  digitalWrite(IN2, LOW);   // Set direction
  analogWrite(ENA, 0);      // Set initial speed (0-255)

  previous_time = millis(); // Record the start time
}

void loop() {
  MoveFWwithGyro();

  if (detectPillar()) {
    int size = calculatePillarSize();
    if (size <= 1000) {  // If the pillar is within the acceptable size range to handle
      avoidPillar();
    }
  }

  if (usForward() < 40 && (usRight() > 70 || usLeft() > 70)) {
    if (usRight() > 70) {
      turnRight();
    } else if (usLeft() > 70) {
      turnLeft();
    }
  }
}

void avoidPillar() {
  int pillarType = detectPillar();
  if (pillarType == 2) { // Red pillar
    turnRightAroundPillar();
  } else if (pillarType == 4) { // Green pillar
    turnLeftAroundPillar();
  }
}

void turnRightAroundPillar() {
  // Change the robot's angle by a small amount to move around the pillar
  targetYawAngle -= 30; 
  int c = 0;
  while (c < 1000) {
    MoveFWwithGyro();
    c++;
  }
  
  // Return the robot to the original path
  targetYawAngle += 30; 
  c = 0;
  while (c < 1000) {
    MoveFWwithGyro();
    c++;
  }
}

void turnLeftAroundPillar() {
  // Change the robot's angle by a small amount to move around the pillar
  targetYawAngle += 30;
  int c = 0;
  while (c < 1000) {
    MoveFWwithGyro();
    c++;
  }
  
  // Return the robot to the original path
  targetYawAngle -= 30;
  c = 0;
  while (c < 1000) {
    MoveFWwithGyro();
    c++;
  }
}

// Functions Definitions

// Move forward with Gyro Feedback
void MoveFWwithGyro() {
  long current_time = millis();
  currentTime = millis();
  float elapsed_time = (current_time - previous_time) / 1000.0; // Calculate elapsed time in seconds
  previous_time = current_time;

  // Get the Z-axis rotation value
  gz = mpu.getRotationZ();

  // Apply drift correction
  gz -= gyro_z_offset;

  // Convert gyroscope reading to degrees per second
  float gyro_z = gz / 131.0;

  // Calculate the yaw angle by integrating the angular velocity over time
  yaw_angle += gyro_z * elapsed_time;
  error = yaw_angle - targetYawAngle;

  // Calculate integral and derivative
  integral += error * (currentTime - previousTime);
  derivative = (error - previousError) / (currentTime - previousTime);

  // Calculate the control signal
  controlSignal = Kp * error + Ki * integral + Kd * derivative;

  float t = controlSignal + 90;

  t = constrain(t, 50, 135);   // Keep the angle within bounds
  myServo.write(int(t));

  previousError = error;
  previousTime = currentTime;
}

// Gyro Calibration
void calculateGyroDrift() {
  for (int i = 0; i < calibration_count; i++) {
    gz = mpu.getRotationZ();
    gyro_z_offset += gz;
    delay(2); // Short delay between readings to stabilize
  }
  gyro_z_offset /= calibration_count;
}

// Ultrasonic Sensor Functions
long getUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);

  long distance = duration * 0.034 / 2;

  return distance;
}

long usLeft() {
  return getUltrasonicDistance(trigPinl, echoPinl);
}

long usRight() {
  return getUltrasonicDistance(trigPinr, echoPinr);
}

long usForward() {
  return getUltrasonicDistance(trigPinf, echoPinf);
}

// Pillar Detection Functions
int detectPillar() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 2) {
        Serial.println("Red pillar detected");
        return 2; // Red pillar detected
      } else if (pixy.ccc.blocks[i].m_signature == 4) {
        Serial.println("Green pillar detected");
        return 4; // Green pillar detected
      }
    }
  }
  
  return 0; // No pillar detected
}

int calculatePillarSize() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 2 || pixy.ccc.blocks[i].m_signature == 4) {
        return pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height; // Return the area of the pillar
      }
    }
  }
  
  return 0; // No pillar detected
}

// Turn Functions
void turnLeft() {
  targetYawAngle += 90; // Adjust angle to the left
  int c = 0;
  while (c < 1500) {
    MoveFWwithGyro();
    c++;
  }
}

void turnRight() {
  targetYawAngle -= 90; // Adjust angle to the right
  int c = 0;
  while (c < 1500) {
    MoveFWwithGyro();
    c++;
  }
}
