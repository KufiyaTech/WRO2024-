#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// .....................................Pin Definitions...........................................//
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

//.........................................variables.............................................//
int16_t gz;
long previous_time;
float yaw_angle = 0;
float gyro_z_offset = 0;
int calibration_count = 2000;                         // Number of samples for calibration
int currentAngle = INITIAL_ANGLE;

unsigned long lastTime = 0;
int i = 0; 
int targetYawAngle = 0;

// PID parameters
float Kp = 1.5;
float Ki = 0.01;
float Kd = 0.5;
// PID variables
float error;
float controlSignal;

int steering_angle  = 89 ;
int c = 0;
int turn_count = 0; // Number of turns

void setup() {
    // ..................................Initialization........................................//
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
    // ..................................Motor Driver setup......................................//
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  
  //................................. Ultrasonic sensor setup..................................//
  pinMode(trigPinl, OUTPUT);
  pinMode(echoPinl, INPUT);
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);
  pinMode(trigPinf, OUTPUT);
  pinMode(echoPinf, INPUT);
  //.............................Check if MPU6050 is connected properly.......................//
  Serial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");
  //.............................Calibrate the gyro to calculate drift........................//
    myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);
  Serial.println("Calibrating gyro...");
  calculateGyroDrift();
  Serial.println("Done Calibrating");
  //.......................................Servo setup.......................................//


  digitalWrite(IN1, HIGH);  // Set direction
  digitalWrite(IN2, LOW);   // Set direction
  analogWrite(ENA, 0);      // Set initial speed (0-255)

  //...................................... Main code .......................................//
  previous_time = millis(); // Record the start time


  while (usForward() > 80 || (usRight() < 70 && usLeft() < 70 )) { 
    MoveFWwithGyro();
    analogWrite(ENA, 255); 
  }
  while (usForward() > 60 || (usRight() < 70 && usLeft() < 70 )) { 
    MoveFWwithGyro();
    analogWrite(ENA, 220); 
  }
  analogWrite(ENA, 0);  
  delay(100);
  
  //....... Determine the Direction of Rotation .......//
  if (usLeft() < usRight()) {  // Clockwise (CW)
    i = 1; 
  } else if (usLeft() > usRight()) { // Counterclockwise (CCW)
    i = 2;
  } else {
    while (1) {
     myServo.write(30);
     delay(1000);
     myServo.write(160);
     delay(1000);
    }
  }
  analogWrite(ENA, 255);
  while (i == 1) {
    analogWrite(ENA, 255);
    MoveFWwithGyro();
    
    if (usForward() < 60 && usRight() > 100) {
      c = 0;
      targetYawAngle -= 90;
      while (c<2000) {
        MoveFWwithGyro();
        c = c+1;
      }
      
      turn_count++; // Increment the turn count
      if (turn_count >= 12) { // Check if 12 turns have been made
        delay(100); // Delay for 1 second before stopping
        analogWrite(ENA, 0); // Stop the robot
        while (1); // Stop permanently
      }
    }
  }

  while (i == 2) {
    MoveFWwithGyro();
    if (usForward() < 60 && usLeft() > 100) { 
      targetYawAngle += 89;
      c=0;
      while (c < 1500) {
        MoveFWwithGyro();
        c=c+1;
      }

      turn_count++; // Increment the turn count
      if (turn_count >= 12) { // Check if 12 turns have been made
        delay(100); // Delay for 1 second before stopping
        analogWrite(ENA, 0); // Stop the robot
        while (1); // Stop permanently
      }
    }
  }
}

void loop() {
  //MoveFWwithGyro();
}

// .....................................Functions Definitions...........................................//
//........... Moving forward with Gyro Feedback ...........//
void MoveFWwithGyro() {
    long current_time = millis();
    
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

  // Calculate the control signal
  controlSignal = Kp * error;

    float t = controlSignal + 90;

    t = constrain(t, 50, 135);   // Keep the angle within bounds
    myServo.write(int(t));
}


void yaw() {
  long current_time = millis();
  float elapsed_time = (current_time - previous_time) / 1000.0; // Calculate elapsed time in seconds
  previous_time = current_time;

  // Get Z-axis rotation value
  gz = mpu.getRotationZ();

  // Apply drift correction
  gz -= gyro_z_offset;

  // Convert gyroscope reading to degrees per second
  float gyro_z = gz / 131.0;

  // Calculate the yaw angle by integrating the angular velocity over time
  yaw_angle += gyro_z * elapsed_time;
}

//................. Gyro Calibration ....................//
void calculateGyroDrift() {
  for (int i = 0; i < calibration_count; i++) {
    // Get Z-axis rotation value
    gz = mpu.getRotationZ();
    gyro_z_offset += gz;
    delay(2); // Short delay between readings to stabilize
  }
  // Average the readings to get the drift (offset) value
  gyro_z_offset /= calibration_count;
}

//................. Ultrasonic....................//
long getUltrasonicDistance(int trigPin, int echoPin) {
  // Trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pin
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in cm
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
