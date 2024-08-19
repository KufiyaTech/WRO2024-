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
    // ..................................Initialization........................................//
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  pixy.init();
  
  // Turn on the Pixy2 camera lights
  pixy.setLamp(1, 0); // Turn on white LEDs

  // ..................................Motor Driver setup......................................//
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  
  //................................. Ultrasonic sensor setup..................................//
  

  //.............................Check if MPU6050 is connected properly.......................//
  Serial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connection failed");

  //.............................Calibrate the gyro to calculate drift........................//
  Serial.println("Calibrating gyro...");
  calculateGyroDrift();
  Serial.println("Done Calibrating");

  // Declare the variables to store pillar detection information
  String pillarColor;
  String pillarSide;
  int pillarSize;

  // Test Pixy2 camera detection
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks) {
    pillarColor = detectPillarColor();
    pillarSide = detectPillarSide();
    pillarSize = detectPillarSize();

    // Print the detected information
    Serial.print("Pillar Color: ");
    Serial.println(pillarColor);
    Serial.print("Pillar Side: ");
    Serial.println(pillarSide);
    Serial.print("Pillar Size: ");
    Serial.println(pillarSize);
  } else {
    Serial.println("No blocks detected.");
  }

  //.......................................Servo setup.......................................//
  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);

  digitalWrite(IN1, HIGH);  // Set direction
  digitalWrite(IN2, LOW);   // Set direction
  analogWrite(ENA, 0);      // Set initial speed (0-255)
 

  //...................................... Main code .......................................//
  previous_time = millis(); // Record the start time
  analogWrite(ENA, 155);


  // Implement logic based on detected pillar characteristics
  while (true) {

    MoveFWwithGyro();

    pillarSide = detectPillarSide(); // Update pillarSide within the loop
    pillarColor = detectPillarColor();
    pillarSize = detectPillarSize();

    if (pillarSize > 1000) {
      stopmotor();
      while (pillarColor == "red") {
        while (pillarSide == "center") {
          myServo.write(0);
          pillarSide = detectPillarSide(); // Continuously update the pillar side
        }
        MoveFWwithGyro();
        pillarColor = detectPillarColor(); // Update the pillar color after moving forward
      }
    }
  }
} 

void loop() {
  // Empty loop function to satisfy the Arduino framework
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

//................. Gyro Calibration ....................//
void calculateGyroDrift() {
  for (int i = 0; i < calibration_count; i++) {
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

// Pixy2 functions
String detectPillarColor() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1) {
        return "red";
      } else if (pixy.ccc.blocks[i].m_signature == 2) {
        return "green";
      }
    }
  }
  return "none";
}

String detectPillarSide() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    int leftBoundary = pixy.frameWidth / 3;          // Left region ends at 1/3 of the frame width
    int rightBoundary = 2 * pixy.frameWidth / 3;     // Right region starts at 2/3 of the frame width

    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int blockX = pixy.ccc.blocks[i].m_x; // x-coordinate of the detected block

      if (blockX < leftBoundary) {
        return "left"; // Object is in the left region
      } else if (blockX > rightBoundary) {
        return "right"; // Object is in the right region
      } else {
        return "center"; // Object is in the center region
      }
    }
  }
  return "none";
}

int detectPillarSize() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    int maxArea = 0;
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
      if (area > maxArea) {
        maxArea = area;
      }
    }
    return maxArea;
  }
  return 0;
}

void stopmotor() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
