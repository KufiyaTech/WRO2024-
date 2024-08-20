#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <Pixy2.h>

// .....................................Pin Definitions...........................................//
#define SERVO_PIN 8
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
float Kp = 1.3;
float Ki = 0;
float Kd = 0.3;
// PID variables
float error;
float previousError = 0;
float integral = 0;
float derivative;
float controlSignal;
unsigned long previousTime;
unsigned long currentTime;

void setup() {
  pixy.init();
  
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
  Serial.println("Calibrating gyro...");
  calculateGyroDrift();
  Serial.println("Done Calibrating");
  
  //.......................................Servo setup.......................................//
  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);

  digitalWrite(IN1, HIGH);  // Set direction
  digitalWrite(IN2, LOW);   // Set direction
  analogWrite(ENA, 0);      // Set initial speed (0-255)

  //...................................... Main code .......................................//
  previous_time = millis(); // Record the start time

  analogWrite(ENA, 255);
  while (usForward() > 40) { 
    MoveFWwithGyro(); 
  }
  
  analogWrite(ENA, 0);  
  delay(100);
  
  //....... Determine the Direction of Rotation .......//
  if (usLeft() < usRight()) {  //CW
    i = 1; 
  } else if (usLeft() > usRight()) { //CCW
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
    int pillarType = detectPillar();
    int size = calculatePillarSize();
    String position = getPillarPosition();

    MoveFWwithGyro();
    if (usForward() < 50 && usRight() > 70) { 
      targetYawAngle -= 90;
      while (yaw_angle > targetYawAngle + 10) {
        MoveFWwithGyro();
      }
    }
    
    while (pillarType == 0) { // "No pillar detected"
      MoveFWwithGyro();
    }
    
    while (pillarType == 1) { // "Red pillar detected"
      if (size < 8000) {
        MoveFWwithGyro();
      } else if (size > 8000) {
        stopMotor();
        while (position == "Left") {
          targetYawAngle += 30;
          break;
        }
        while (position == "Right") {
          targetYawAngle -= 45;
          break;
        }
        while (position == "Center") {
          targetYawAngle += 0;
          break;
        }
        MoveFWwithGyro();
      }
    }
    while (pillarType == 2 ){
      if (size < 8000) {
        MoveFWwithGyro();
      } else if (size > 8000) {
        stopMotor();
        while (position == "Left") {
          targeYawAngle += 30;
          break;
        }
        while (position == "Right") {
          targetYawAngle -= 45;
          break;
        }
        while (position == "Center") {
          targetYawAngle += 0;
          break;
        }
        MoveFWwithGyro();
      }
    }
    }
  }

  while (i == 2) {
    MoveFWwithGyro();
    if (usForward() < 50 && usLeft() > 70) { 
      targetYawAngle += 90;
      while (yaw_angle < targetYawAngle - 10) {
        MoveFWwithGyro();
      }
    }
  }
  while (pillarType == 0) { // "No pillar detected"
      MoveFWwithGyro();
    }
    
    while (pillarType == 1) { // "Red pillar detected"
      if (size < 8000) {
        MoveFWwithGyro();
      } else if (size > 8000) {
        stopMotor();
        while (position == "Left") {
          targetYawAngle += 30;
          break;
        }
        while (position == "Right") {
          targetYawAngle -= 45;
          break;
        }
        while (position == "Center") {
          targetYawAngle += 0;
          break;
        }
        MoveFWwithGyro();
      }
    }
    while (pillarType == 2 ){
      if (size < 8000) {
        MoveFWwithGyro();
      } else if (size > 8000) {
        stopMotor();
        while (position == "Left") {
          targeYawAngle += 30;
          break;
        }
        while (position == "Right") {
          targetYawAngle -= 45;
          break;
        }
        while (position == "Center") {
          targetYawAngle += 0;
          break;
        }
        MoveFWwithGyro();
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

  t = constrain(t, 40, 140);   // Keep the angle within bounds
  myServo.write(int(t));

  previousError = error;
  previousTime = currentTime;
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

int detectPillar() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1) {
        return 1; // Red pillar detected
      } else if (pixy.ccc.blocks[i].m_signature == 2) {
        return 2; // Green pillar detected
      }
    }
  }
  
  return 0; // No pillar detected
}

int calculatePillarSize() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1 || pixy.ccc.blocks[i].m_signature == 2) {
        return pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height; // Return the area of the pillar
      }
    }
  }
  
  return 0; // No pillar detected
}

String getPillarPosition() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1 || pixy.ccc.blocks[i].m_signature == 2) {
        int x = pixy.ccc.blocks[i].m_x;
        if (x < 100) {
          return "Left";
        } else if (x > 220) {
          return "Right";
        } else {
          return "Center";
        }
      }
    }
  }
  
  return "No Pillar"; // No pillar detected
}

void stopMotor() {
  // Set both IN1 and IN2 to LOW to stop the motor
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  
  // Set ENA to 0 to ensure no power is being delivered
  analogWrite(ENA, 0);
}
