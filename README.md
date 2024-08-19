# Hi, we are KufiyaTech team , representing our beloved country , Palestine!
<p align="center">
  <img src="https://github.com/KufiyaTech/WRO2024-/assets/172860664/762866fa-9850-4595-868a-0e80bc71700c" alt="Palestine Flag" width="200"/>
  &nbsp;&nbsp;&nbsp;&nbsp;
  <img src="https://github.com/KufiyaTech/WRO2024-/assets/172860664/062a5ed9-cc8a-4399-9fc4-3b65bad85b79" alt="Purpose Logo" width="200"/>
</p>

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2024.

## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## READ.ME table of content

1. [Project Overview](#project-overview)

   1.1. [Competition Context](#competition-context)
   
   1.2. [Team Management](#team-management)
   
2. [How Does Our Robot Think](#how-does-our-robot-think)
 
   2.1. [Open Challenge Overview](#open-challenge-overview)
   
   2.2. [Obstacle Challenge Overview](#obstacle-challeng-overview)
   
   2.3. [Moving at Safe Zone](#moving-at-safe-zone)
   
   2.4. [Detecting Turns and Direction](#detecting-turns-and-direction)
   
   2.5. [Lap Counting Mechanism](#lap-counting-mechanism)
   
   2.6. [IMU-Based Steering](#imu-based-steering)
   
3. [Open Challenge Algorithm](#open-challenge-algorithm)

   3.1. [PD Controller](#pd-controller)
   
   3.2. [Turn Execution](#turn-execution)
   
4. [Obstacle Avoidance Round Challenge](#obstacle-avoidance-round-challenge)

   4.1. [Pillar Detection](#pillar-detection)
    
   4.2. [Obstacle Avoidance Strategy](#obstacle-avoidance-strategy)
   
5. [Designing Process](#designing-process)

   5.1. [Steering System](#steering-system)
    
   5.2. [Differential Gear](#differential-gear)
   
   5.3. [Chassis](#chassis)
   
   5.4. [Mechanism](#mechanism)
   
   5.5. [Ackermann Steering Mechanism](#ackermann-steering-mechanism)
   
6. [Power and Sense Management](#power-and-sense-management)

   6.1. [Power Source](#power-source)
    
   6.2. [Sensors We Used and Their Functions](#sensors-we-used-and-their-functions)
   
7. [Appendices](#appendices)

   7.1. [Datasheets and Specifications](#datasheets-and-specifications)
    
   7.2. [Code Listings](#code-listings)
   
   7.3. [Additional Diagrams](#additional-diagrams)
   
   7.4. [References](#references)
   
8. [Hurdles and Challenges](#hurdles-and-challenges)

   8.1. [Designing Process Challenges](#designing-process-challenges)
    
   8.2. [Sensor and Coding Challenges](#sensor-and-coding-challenges)
   
   8.3. [General Challenges](#general-challenges)
   
9. [Conclusion and Reflections](#conclusion-and-reflections)
 
   9.1. [Lessons Learned](#lessons-learned)
    
   9.2. [Future Work](#future-work)



## Project Overview
This project is developed for the WRO Future Engineers 2024 competition and focuses on designing and programming an autonomous robotic car. The car is capable of navigating a white map marked with distinct blue and orange lines, utilizing a combination of advanced sensors such as the TCS3200 for color detection and an IMU for precise turning. The primary goal is to optimize the car's performance during the competition, specifically excelling in the obstacle challenge and open challenge rounds.

The competition requires a sophisticated approach to both hardware and software. We’ve integrated robust algorithms that allow the car to autonomously make decisions based on real-time data from its sensors. This not only showcases the car's ability to navigate and avoid obstacles but also emphasizes its capacity to follow designated paths with high accuracy. The project encompasses several engineering disciplines, including electronics, mechanics, and software development, making it a comprehensive example of modern engineering applied to robotics.


### 1.1 Competition Context

The WRO Future Engineers 2024 competition challenges teams to design and develop autonomous vehicles capable of navigating a predefined course while executing specific tasks. This competition is structured across multiple rounds, including a practice round, two qualification rounds, and a final round, each progressively testing the vehicle's capabilities.

Participants must engineer their vehicles to handle complex challenges such as lane following, obstacle detection, and precise turning. The competition track is designed with a variety of elements, including straight paths, curves, and intersections, which the vehicle must navigate autonomously without human intervention. 

Scoring is based on the vehicle's ability to complete the course with accuracy and efficiency, with additional points awarded for successful task execution and speed.

For detailed competition rules, please refer to the official WRO Future Engineers 2024 guidelines: [WRO Future Engineers 2024 Rules](https://wro-association.org/wp-content/uploads/WRO-2024-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)

### 1.2 Team Management

Our project is the result of a collaborative effort by a skilled and dedicated team, each member bringing unique expertise to the table:

- **Sara Jawaada**: As the lead designer, Sara was instrumental in overseeing the entire design process, ensuring that the physical and conceptual elements of the project were seamlessly integrated. Additionally, she played a key role in software development, contributing to both the architecture and implementation of core functionalities.

**Contact Information**:
  **Email**: [sarajawaada906@gmail.com](sarajawaada906@gmail.com)


- **Amro Duhaidi**: Amro led the technical implementation of the software, with a particular focus on sensor integration and system wiring. His expertise in coding and hardware interfacing was crucial in developing the responsive and reliable behavior of the robot, ensuring that all sensors were effectively utilized in the project's logic.

**Contact Information**:
  **Email**: [amro.duaidi@gmail.com](amro.duaidi@gmail.com)


- **Tala Daraghmeh**: Tala managed the version control and documentation of the project, ensuring that all development efforts were well-documented and accessible via GitHub. Her contributions also extended to software development, where she collaborated closely with the team to refine the codebase and maintain project integrity.

**Contact Information**:
  **Email**: [taladaraghmeh836@gmail.com](taladaraghmeh836@gmail.com)

**Acknowledgment**

We would like to extend our deepest gratitude to our mentor and coach, **Eng. Mohammad Muamar**. A distinguished engineer and graduate of Palestine Polytechnic University in Hebron, Eng. Muamar has been instrumental in guiding our team through every step of this competition. His dedication, expertise, and unwavering support have been invaluable to our progress and success.

As students of esteemed professors who are proud holders of Palestinian degrees with expertise of global standards, we are honored to have Eng. Muamar as our mentor. His commitment to excellence and his belief in our potential have profoundly shaped our approach and achievements in this project.

**Contact Information**:
- **Email**: [moh.mummar@gmail.com](mailto:moh.mummar@gmail.com)

**KufiyaTech Team's Social Accounts**

Stay connected with our team and follow our journey through our official social media channels. Explore more about our work, updates, and behind-the-scenes moments by visiting our link tree:

- **KufiyaTech Social Accounts**: [https://linktr.ee/kufiyatech](https://linktr.ee/kufiyatech)


## 2. How Does Our Robot Think

To ensure our vehicle can successfully complete the tasks required in both rounds of the competition, we have utilized a variety of sensors, each with a specific function, as detailed in the [Power and Sense Management](#power-and-sense-management) section of this README.

Our robot is powered by an Arduino Mega microcontroller and is programmed in **C++**. We chose C++ as our programming language due to its high performance and efficiency, which are crucial for real-time processing and control in embedded systems like ours. Additionally, C++ provides fine-grained control over system resources, which is essential for optimizing the performance of our robot’s algorithms.

The core logic of the vehicle relies on a PD (Proportional-Derivative) controller to navigate within the designated safe zone. The vehicle is equipped with three ultrasonic sensors and an IMU (Inertial Measurement Unit) sensor, which together manage the steering system effectively. Additionally, a front-facing ultrasonic sensor is employed to detect U-turns. This combination of sensors and control algorithms forms the backbone of our robot's decision-making process, allowing it to navigate the course autonomously and efficiently.

**Programming Language and Libraries**

 **Programming Language**: **C++**

- **Why C++?**:

- We chose C++ because it is well-supported by the Arduino IDE and provides robust library support, making it an ideal choice for our project.
  
We leveraged several libraries to implement the functionalities required for our robot:

- **`Wire.h`**: Facilitates I2C communication between the Arduino and various sensors, such as the IMU, allowing us to efficiently gather and process sensor data.

- **`Servo.h`**: Used for controlling the servo motors, particularly in steering and managing precise movements.

- **`Pixy2.h`**: Handles communication with the Pixy2 camera sensor, which is crucial for visual tracking and object recognition tasks.

- **`MPU6050.h`**: Provides functions to interface with the MPU6050 IMU sensor, enabling accurate detection of orientation, acceleration, and gyroscopic data to manage the robot's movement and stability.

These libraries, combined with the robustness of C++, provide the necessary tools to implement the complex control logic required by our autonomous vehicle.

### 2.1 Open Challenge Overview

**Round objectives:**

•	Moving between the internal and external wall.

•	Turning the  car when detecting the lines . 

•	Detecting blue and orange lines on the mat.

•	Counting the laps.

•	The car stops after 3 laps. 

**Round constraints:**

•	Time.

•	The car Turning in the correct angle.

•	The direction of the car's movement is random.

•	he position from which the car starts moving is random.

•	The distance between the internal and external wall is random.

### 2.2 Obstacle Challenge Overview

**Round objectives:**

•	Detecting red and green pillars.

•	Detecting orange and blue lines.

•	Turn right if detecting red pillar.

•	Turn left if detecting green pillar.

•	Turn when detecting blue or orange line.

•	Move straight between the internal and external walls.

**Round constraints:**

•	The positions of the pillars are random.

•	The number of the pillars is random.

### 2.3 Moving at safe zone
### 2.4 Detecting Turns and Direction

**Problem Statement:**
In the WRO Future Engineers competition, the direction of the car's movement is not predetermined; it can be either clockwise or counterclockwise. This unpredictability introduces an additional layer of complexity, particularly when the car needs to execute precise U-turns at specific points on the track. The challenge is to accurately detect the required direction and execute the turn with precision.

**First Solution: Using the TCS3200 Color Sensor**

One approach to solving this problem involved the use of the TCS3200 color sensor to detect the color-coded lines on the track that indicate the direction of the car's movement.

- **Color Detection:** The TCS3200 sensor was programmed to recognize specific colors on the track. An orange line signals that the car should move in a clockwise direction, while a blue line indicates a counterclockwise movement.

- **Controlled Turning:** Upon detecting a color, the car was instructed to turn at a specific angle. The turning angle was precisely controlled using an Inertial Measurement Unit (IMU), ensuring that the turn was executed accurately and consistently, regardless of external factors like speed or track conditions.

- **Code Implementation:** The process was automated through predefined functions such as `detectOrangeLine` and `detectBlueLine`, which triggered the appropriate turning actions based on the color detected.

This method provided a clear and direct way to determine the car's direction based on visual cues from the track, leveraging the IMU for precision in turning.

**Second Solution: Using Three Ultrasonic Sensors**

An alternative approach utilized three ultrasonic sensors to determine the direction of the car's movement based on distance measurements:

- **Direction Determination:** The ultrasonic sensors were positioned to monitor distances on the left, right, and front of the car. When the car reached a point where it needed to turn, the direction was determined by comparing the distances detected by the sensors:
  - If the distance measured by the right sensor was greater than that of the left sensor, the car would turn clockwise.
  - Conversely, if the left distance was greater, the car would turn counterclockwise.

- **Front Sensor Trigger:** Additionally, the front ultrasonic sensor played a crucial role by detecting when the car was approximately 50 cm away from an obstacle or turning point. This detection served as a cue for the car to initiate the turn.

This method relied on spatial awareness and distance measurement, allowing the car to make decisions based on its surroundings without relying on visual markers.

<p align="center">
  <img src="https://github.com/user-attachments/assets/0fac0299-f811-430a-a7f3-c70c11bd5935" alt="Turn Detection Logic" width="400"/>
</p>
<p align="center">
  <em>Figure 1: Illustration of the turn detection logic using ultrasonic sensors.</em>
</p>

As illustrated above, when the car reaches the lines where it needs to turn, the direction of the turn is determined by the distance sensors. If the sensors detect that the right distance is greater than the left distance, the car should turn clockwise. If the left distance is greater than the right distance, the car will move counterclockwise. When the front sensor detects that the distance is approximately 50 cm, it indicates that the car should turn.

**Final Decision: Implementing the Ultrasonic Sensor-Based Approach**

After careful consideration, the "Kufiya" team decided to implement the second method using ultrasonic sensors. This decision was driven by numerous challenges encountered with the first method involving the color sensor. Despite spending two months attempting to fine-tune the color sensor, we faced significant issues with its accuracy, particularly in varying lighting conditions and with precise color recognition.

The sensor's inconsistent performance led to unreliable direction detection, which was critical for the success of our project. These difficulties prompted us to shift our focus to the ultrasonic sensor-based approach, which provided a more reliable and adaptable solution for determining the car's movement direction and executing precise turns.

### 2.5 Lap Counting Mechanism

**Problem Statement:**
The goal is to implement a lap counting mechanism for an autonomous vehicle (or RC car) that accurately counts the number of laps the vehicle completes on a designated track. A key requirement is that after completing exactly three laps, the vehicle must automatically stop.

#### 1. Color Sensor-Based Detection

**Overview:**
The first method involves utilizing a color sensor (TCS3200) to detect specific markers placed on the track. This technique leverages the sensor's ability to identify distinct colors, allowing it to register a lap each time the sensor passes over a designated colored marker.

**Principle:**
- The color sensor is calibrated to recognize a specific color distinct from the track's surface.
- When the sensor detects this color, it triggers a signal to increment the lap count.

**Code Example:**
*(Include a relevant code snippet here to demonstrate how the color sensor was implemented)*

**Why Do We Not Recommend This Method?**

- **Accuracy Issues:** Despite extensive efforts, the color sensor consistently struggled to provide accurate readings, particularly with the color orange. Ambient lighting conditions significantly affected the sensor's performance, even with a custom shield.
- **Programming Challenges:** Programming the sensor to reliably identify the target color was difficult, especially given the high speed of the vehicle, which impaired the sensor’s ability to detect and register the color in real-time.
- **Unreliable Performance:** The sensor's inconsistent and imprecise measurements, especially under dynamic conditions, made it an unsuitable choice for our project. This unreliability led us to explore alternative lap counting methods.

#### 2.5 Loop-Based Counting with Ultrasonic Sensor and IMU

**Overview:**
The second method utilizes a software-based loop counter that increments the lap count each time a specific condition within the code is met. In our approach, the robot detects the completion of a lap using an ultrasonic sensor. When the sensor detects that the distance to a specific point or object is less than 50 cm, the robot recognizes this as an indicator to begin a new lap. The robot’s steering is controlled by an IMU (Inertial Measurement Unit), which ensures precise navigation during turns.

**Principle:**
- The loop-based method is implemented by continuously monitoring the distance using the ultrasonic sensor.
- When the sensor detects that the distance has decreased below 50 cm, the robot initiates a turning maneuver, guided by the IMU.
- This detection and turning process is managed within a loop structure in the code, specifically within a `while` loop that executes as long as the distance remains below 60 cm.
- Each time this condition is met, the program increments the lap count, allowing for accurate tracking of the robot’s laps.

**Advantages of This Method:**
- **Reliability:** This method provides more reliable lap counting, unaffected by external factors like ambient light.
- **Precision:** The IMU ensures precise control of the vehicle’s steering, contributing to accurate lap counting



---

### 2.6 IMU-Based Steering

**Overview:**

In our autonomous vehicle, precise steering control is essential for navigating the course accurately. We implemented an IMU-based steering mechanism to achieve this. The IMU (Inertial Measurement Unit) provides real-time data about the vehicle’s angular velocity, which is critical for calculating the yaw angle (the vehicle’s turning angle) and ensuring stable and accurate steering.

**Understanding the IMU:**

An IMU (Inertial Measurement Unit) is a sensor device that combines multiple sensing components, usually a 3-axis accelerometer and a 3-axis gyroscope. These sensors work together to track the vehicle's motion and orientation in three-dimensional space.

- **Accelerometer**: Measures the acceleration along the X, Y, and Z axes. It helps in determining the tilt of the vehicle.
- **Gyroscope**: Measures the angular velocity (the rate of rotation) around the X, Y, and Z axes. It is crucial for determining the vehicle's rotational motion, especially the yaw angle.

**The X, Y, and Z Axes:**

Understanding the axes is essential for interpreting the data provided by the IMU:

- **X-Axis**: Represents the roll, or the tilting motion of the vehicle from side to side.
- **Y-Axis**: Represents the pitch, or the tilting motion of the vehicle from front to back.
- **Z-Axis**: Represents the yaw, or the rotational movement of the vehicle around the vertical axis. This axis is crucial for steering, as it defines the direction the vehicle is facing.

![Axis Diagram](![image](https://github.com/user-attachments/assets/9a754548-b259-41bb-bc29-a1b27a1d6bb7)
)
*Figure 1: The X, Y, and Z axes relative to the vehicle.*

**Calculating the Yaw Angle:**

The yaw angle (\(\theta_z\)) is a measure of the vehicle's rotation around the Z-axis. The gyroscope within the IMU provides angular velocity data along this axis, which tells us how quickly the vehicle is rotating.

To determine the vehicle's orientation or heading, we calculate the yaw angle by integrating the angular velocity over time. The yaw angle changes based on the rate of rotation, and by continuously updating this value, we can keep track of the vehicle's direction.

The equation used to calculate the yaw angle is:

\[
\theta_z = \theta_{z,\text{previous}} + \left( \frac{\text{gyro}_z - \text{gyro\_z\_offset}}{131.0} \right) \times \Delta t
\]


Where:
- \(\theta_z\) is the current yaw angle.
- \(\theta_{z, \text{previous}}\) is the yaw angle from the previous time step.
- \(\text{gyro}_z\) is the angular velocity around the Z-axis (provided by the gyroscope).
- \(\text{gyro\_z\_offset}\) is the gyroscope offset, calculated during calibration to correct any drift.
- \(\Delta t\) is the time elapsed between the current and previous readings.

**Importance of Gyro Offset:**

The gyroscope offset (\(\text{gyro\_z\_offset}\)) is critical because gyroscopes can have slight errors or biases over time, known as drift. By calculating and subtracting this offset, we ensure that the yaw angle calculation is accurate and doesn't gradually deviate from the true value.

**PID Control for Steering:**

To maintain or correct the vehicle’s path, we implemented a PID (Proportional-Integral-Derivative) controller. The PID controller adjusts the steering angle based on the difference between the target yaw angle and the current yaw angle. This ensures smooth and stable steering.

- **Proportional (P)**: Corrects the yaw angle based on the current error (difference between the desired and actual yaw angle).
- **Integral (I)**: Accumulates past errors to eliminate steady-state offset, ensuring the vehicle reaches and maintains the target angle.
- **Derivative (D)**: Predicts future error based on the rate of change, helping to reduce overshoot and oscillations.

The control signal, which is sent to the servo motor, is calculated as:

\[
\text{controlSignal} = K_p \times \text{error} + K_i \times \int \text{error} \, dt + K_d \times \frac{d(\text{error})}{dt}
\]

Where:
- \(K_p\), \(K_i\), and \(K_d\) are the PID coefficients, which determine how aggressively the controller responds to the error.
- \(\text{error}\) is the difference between the target and actual yaw angles.

![PID Control Block Diagram](https://github.com/user-attachments/assets/a3b33c40-0745-46f0-8fc4-7972a4d19d81)
*Figure 2: PID Control System for Steering.*

### Code Implementation:

The code implementation ties together the IMU readings, yaw angle calculation, and PID control to manage the vehicle's steering:

```cpp
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo myServo;
int16_t gz;
float yaw_angle = 0, gyro_z_offset = 0;
float Kp = 2, Ki = 0, Kd = 0.5;
float error, integral = 0, derivative, controlSignal;
unsigned long previousTime, currentTime;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();
    myServo.attach(SERVO_PIN);
    Serial.println("Calibrating gyro...");
    calculateGyroDrift();
    Serial.println("Done Calibrating");
    previousTime = millis();
}

void loop() {
    MoveFWwithGyro();
}

void MoveFWwithGyro() {
    currentTime = millis();
    float elapsed_time = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    gz = mpu.getRotationZ() - gyro_z_offset;
    float gyro_z = gz / 131.0;
    yaw_angle += gyro_z * elapsed_time;

    error = targetYawAngle - yaw_angle;
    integral += error * elapsed_time;
    derivative = (error - previousError) / elapsed_time;
    controlSignal = Kp * error + Ki * integral + Kd * derivative;
    controlSignal = constrain(controlSignal, 40, 140);
    myServo.write(int(controlSignal));

    previousError = error;
    Serial.print("Yaw Angle: "); Serial.println(yaw_angle);
}

void calculateGyroDrift() {
    for (int i = 0; i < 2000; i++) {
        gyro_z_offset += mpu.getRotationZ();
        delay(2);
    }
    gyro_z_offset /= 2000;
}
```

### Explanation of the Code:

1. **Setup:**
   - The IMU and servo motor are initialized. The gyroscope is calibrated to calculate the `gyro_z_offset`.
   
2. **Main Loop:**
   - The function `MoveFWwithGyro()` is called continuously. It reads the gyroscope data, calculates the yaw angle, and then computes the control signal using the PID controller.
   
3. **Yaw Angle Calculation:**
   - The yaw angle is updated based on the angular velocity around the Z-axis, factoring in the time elapsed since the last update.
   
4. **PID Control:**
   - The error between the desired and actual yaw angle is calculated. The control signal, which adjusts the steering angle, is then determined by the PID formula. This control signal is sent to the servo motor to adjust the vehicle's direction.







