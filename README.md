# Hi, we are KufiyaTech team , representing our beloved country , Palestine!
![palestine-flag-illustration-hand-drawn-of-palestine-flag-flag-of-palestine-vector-removebg-preview](https://github.com/KufiyaTech/WRO2024-/assets/172860664/762866fa-9850-4595-868a-0e80bc71700c)![Capture-removebg-preview](https://github.com/KufiyaTech/WRO2024-/assets/172860664/062a5ed9-cc8a-4399-9fc4-3b65bad85b79)







====

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
 
   2.1. [Open Challenge Logic Overview](#open-challenge-logic-overview)
   
   2.2. [Obstacle Challenge Logic Overview](#obstacle-challenge-logic-overview)
   
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


### 1.2 Team Management

Our project is the result of a collaborative effort by a skilled and dedicated team, each member bringing unique expertise to the table:

- **Sara Jawaada**: As the lead designer, Sara was instrumental in overseeing the entire design process, ensuring that the physical and conceptual elements of the project were seamlessly integrated. Additionally, she played a key role in software development, contributing to both the architecture and implementation of core functionalities.

**Contact Information**:
  **Email**: [sarajawaada906@gmail.com](sarajawaada906@gmail.com)


- **Amro Duhaidi**: Amro led the technical implementation of the software, with a particular focus on sensor integration and system wiring. His expertise in coding and hardware interfacing was crucial in developing the responsive and reliable behavior of the robot, ensuring that all sensors were effectively utilized in the project's logic.

**Contact Information**:
  **Email**: [amro.duaidi@gmail.com](amro.duaidi@gmail.com)


- **Tala Daraghmeh**: Tala managed the version control and documentation of the project, ensuring that all development efforts were well-documented and accessible via GitHub. Her contributions also extended to software development, where she collaborated closely with the team to refine the codebase and maintain project integrity.
- 
**Contact Information**:
- **Email**: [taladaraghmeh836@gmail.com](taladaraghmeh836@gmail.com)

### Acknowledgment

We would like to extend our deepest gratitude to our mentor and coach, **Eng. Mohammad Muamar**. A distinguished engineer and graduate of Palestine Polytechnic University in Hebron, Eng. Muamar has been instrumental in guiding our team through every step of this competition. His dedication, expertise, and unwavering support have been invaluable to our progress and success.

As students of esteemed professors who are proud holders of Palestinian degrees with expertise of global standards, we are honored to have Eng. Muamar as our mentor. His commitment to excellence and his belief in our potential have profoundly shaped our approach and achievements in this project.

**Contact Information**:
- **Email**: [moh.mummar@gmail.com](mailto:moh.mummar@gmail.com)

# KufiyaTech team's social accounts

https://linktr.ee/kufiyatech



## Behind the Scenes: Tackling Technical Hurdles in Our Robot Project:

## The Hurdles:
1- While trying to find a solution for the obstacle challenge round, we needed to program the robot to recognize red and green traffic signs. What should we do? We researched and read a lot about the OpenCV library and decided to use it to enable the robot to perform color detection. This was our first experience with image processing. However, we faced many difficulties when using OpenCV; the detection was not accurate. We tried multiple times to fix the issue but eventually decided to abandon OpenCV and use the Pixy2 camera, which is specifically designed for color recognition.

2- Since the mechanism was made of LEGO, we faced issues regarding the sturdiness of the steering and the initial turning angle. When the car hit something, the wheels would fall off. Therefore, we decided to modify the mechanism and used a LEGO kit to make it more robust.

3- Due to the numerous components that needed to be connected to the Arduino, wiring them in a way that prevents interference and arranging and securing them to avoid disconnections was a challenge.
## From Concept to Creation: Innovating Our Robot with Artistry
The WRO rules allowed us to use a ready-made robot structure, but Team Kufiyatech refused the easy route. As we are Equipped with specialized training from the Purpose Academy in FreeCAD design platform, we decided to take the challenging path. We embarked on designing our robot from scratch and 3D-printing it. This journey required meticulous attention and focus; we had to redesign several times until we found the perfect design. This step consumed considerable time and effort from our team, with special thanks to Sara Jawaada, our team member responsible for the design. 
# Mechanism
Your vehicle’s drivetrain works with the engine to deliver power to the wheels. The most common types of drivetrains are front-wheel drive (FWD), rear-wheel drive (RWD), four-wheel drive (4WD) and all-wheel drive (AWD), Our car utilized a rear-wheel drive system.

# Raer-wheel drive(RWD):
![image](https://github.com/KufiyaTech/WRO2024/assets/172860664/97326854-3f39-4af9-92d1-6d75792b20d6)

It is a type of drivetrain system in vehicles where the engine's power is directed to the rear wheels. Rear-wheel drive vehicles have balanced weight distribution and higher control over the vehicle. The rear-wheel drive system consists of several main components: the drive shaft, the rear axle, and the differential gears, which in turn transmit the power to the axles and then to the wheels, Our car's mechanism was constructed using the LEGO EV3 kit.

https://www.amazon.com/Lego-Mindstorm-Ev3-Core-45544/dp/B00DEA55Z8?th=1
# Mastering Precision: Differential and Steering Dynamics:

# Ackermann steering geometry 

we used We used Ackerman steering , Ackermann steering geometry is a configuration designed to ensure that the wheels of a vehicle trace out circles with different radii during a turn, preventing tire slippage. It was invented by Georg Lankensperger and patented by Rudolph Ackermann. This geometry helps align the front wheels towards the turning center, providing improved handling and stability, especially at low speeds. It contrasts with other mechanisms like the Davis steering gear, offering simpler construction and fewer components susceptible to wear. Ackermann steering is commonly used in standard vehicles for its advantages in maneuverability and reduced tire wear​
 ![image](https://github.com/KufiyaTech/WRO2024/assets/172860664/17ccce49-5632-41f0-b007-5583dd16745b) ![image]

# Differential Gearbox

We opted to use a differential gearbox for the rear wheels of the robot , A differential gearbox is a crucial component in vehicles, enabling wheels on the same axle to rotate at different speeds. This functionality is vital during cornering, as it allows the outer wheel to travel a greater distance than the inner wheel, enhancing traction and handling. The device operates through a set of gears that balance the torque distribution between the wheels, ensuring smooth and efficient power transmission. Differential gearboxes are essential in various vehicles, from cars to heavy machinery, optimizing performance and safety.

![image](https://github.com/KufiyaTech/WRO2024/assets/172860664/0ee09c0b-852b-486b-8228-b37931adc5ea)







# KufiyaTech's Engineering Masterpiece: The Chassis Design Story

The design of our robot has evolved over time, and here is how our designs have developed through this journey:


# Steering System:

As we mentioned before, We chose the Ackerman Steering.
# First design:

Our initial steering system was effective in terms of functionality, but we faced challenges when it came to supporting and securing all the components on top of it

![image](https://github.com/KufiyaTech/WRO2024/assets/172860664/b4ffc615-b223-4875-a704-f96b63dd2298)

# Second design:

Opting for this steering system proved to be a better choice initially, but the system was based on ready-to-print files sourced from the internet, which presented us with a dilemma: adhere to the predefined dimensions that we couldn't alter, or undertake the challenge of designing it entirely from scratch.
https://cults3d.com/en/3d-model/gadget/3d-printed-rc-car-with-brushless-motor-lee3d999

# third design:

Unlike our previous designs, this iteration didn't require 3D printing because we utilized the EV3 LEGO kit for the mechanism. This approach saved us time and effort, allowing us the flexibility to modify and control it easily. The LEGO kit provided a reliable steering solution that we could test quickly, contrasting with the longer process required by our previous designs.
                                                                                                                                  


# First design:

Our initial choice was similar to the second option in terms of using ready-to-print files for the steering system. However, we encountered familiar challenges, compounded by an additional difficulty: the inability to locate suitable shafts or bearings within our allotted timeframe.

https://cults3d.com/en/3d-model/gadget/3d-printed-rc-car-with-brushless-motor-lee3d999


# Second design:
In this instance, we opted to utilize the EV3 LEGO system to design and construct our steering mechanism, marking a departure from our previous approaches. This choice offered the same advantages as our third steering system design, particularly in terms of ease of management and reliability

# Chassis:

First design:
Our initial design phase was the most time-consuming. Being our first attempt, we grappled with defining the shape of our robot and determining how it would accommodate all intended components, including five ultrasonic sensors. This constrained our design options significantly. Our primary objective was to create a chassis capable of housing all components, prioritizing functionality over minimizing size. We positioned the camera at the rear without adhering to standard engineering principles. Initially, our plan was to utilize a 3D printer for manufacturing.


# Second design:

Our second design was the first one we brought to life by printing it using a 3D printer. This design incorporated our chosen mechanism, but upon completion, we discovered it was too compact to meet our established standards

# | first design| Second design| 
| ![image](https://github.com/KufiyaTech/WRO2024/assets/172860664/880ac769-6f3d-44d7-a4ef-9e1b801f6266)  |     
| ![image](https://github.com/KufiyaTech/WRO2024/assets/172860664/1bd015c1-421a-4a30-905c-2772ba682bd6)   | 


# third design:

Before proceeding with the computer-aided design, we opted to sketch our third design on paper. We then crafted a prototype using compressed cork to evaluate its dimensions. Our aim was to slightly increase its size while accommodating our chosen mechanisms. However, due to limitations with our printer, we were compelled to design two separate bases (chassis) and join them together. During testing, we discovered a potential vulnerability: the structure could potentially break apart under specific weight conditions.

                                                                                                                                           |
| ---------| ---------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------|
| ![image](https://github.com/KufiyaTech/WRO2024/assets/172860664/a5f15b99-cfae-4cd8-bba8-e09ab30f1281)     | ![image](https://github.com/KufiyaTech/WRO2024/assets/172860664/e78e858e-036b-499c-a5f9-99556fc64504)                   | ![image](https://github.com/KufiyaTech/WRO2024/assets/172860664/c8822e1b-69e6-4e3f-ad21-80fed76dd14e)
| ![image](https://github.com/KufiyaTech/WRO2024/assets/172860664/3407ff2e-6af5-4a1a-9452-e8134866ccaf)       | 


 # fourth design:
 
We decided to shift from using a 3D printer to a CNC machine for bringing our designs to reality. This allowed us to create a single, connected chassis capable of supporting more weight. Our design continued to evolve due to changes in our mechanism and component choices. A significant change was opting for smaller LEGO wheels to improve steering, which took some time to perfect. Another challenge we faced was managing space constraints relative to the size of our mechanisms. However, with the guidance and support of our great coach, we were able to find better solutions and overcome these obstacles.
After deciding to use a CNC machine, we encountered several challenges in selecting the ideal material to meet our needs. Initially, we chose acrylic, but it proved too brittle and broke easily. We then searched for a more suitable material and ultimately selected wood, which met our goals and requirements perfectly.

![image](https://github.com/KufiyaTech/WRO2024/assets/172860664/8d3b3f33-9369-4ae3-a089-77990af68bc5)

![image](https://github.com/KufiyaTech/WRO2024/assets/172860664/27b721c8-c8ca-469b-8199-d90283b693bb)
## How does our robot think?

Using the Arduino Mega 2560 microcontroller, our car is controlled and given commands. The programming was done in C++. Why C++? Because the Arduino Integrated Development Environment (IDE) is built to support C++ (with some simplifications and extensions). The libraries and core functions of the Arduino platform are written in C++. We used a set of sensors, each for a specific function so that the car could complete the required task.

## Open challenge:

Round objectives:

•	Moving between the internal and external wall.
•	Turning the  car when detecting the lines . 
•	Detecting blue and orange lines on the mat.
•	Counting the laps.
•	The car stops after 3 laps. 

Round constraints: 

•	Time.
•	Color sensor accuracy.
•	The car Turning in the correct angle.
•	The direction of the car's movement is random.
•	he position from which the car starts moving is random.
•	The distance between the inner and outer wall is random.

How did Ghoson ( our car ) complete this round?

Open challenge round main loop code logic: For this round, we used 3 ultrasonic sensors to keep the car between the internal and external walls, and the MPU6050 to keep the car moving straight without bumping in the walls, we used the TCS3200 color sensor for detecting the blue and orange lines , if the line that is detected is orange , the direction of the car’s movement is counterclockwise , if the line is blue , the direction is clockwise , when the car detects a line whether its blue or orange , and if the front ultrasonic sensor detects that the front sensor is nearly 100cm it will turn , we adjusted the turning angle using the MPU6050,  when the color sensor counts 12 orange and blue lines then the car has finished 3 laps , the car should stop moving.

## Obstacle avoidance:

Round objectives:

•	Detecting red and green pillars.
•	Detecting orange and blue lines.
•	Turn right if detecting red pillar.
•	Turn left if detecting green pillar.
•	Turn when detecting blue or orange line.
•	Move straight between the internal and external walls.

Round constraints:

•	The positions of the pillars are random.
•	The number of the pillars is random.
How did Ghoson ( our car ) complete this rouned?
Obstacle avoidance challenge main loop code logic: Using the pixy2 camera the car detected the green and the red pillars , we programmed the pixy to devise its frame from the half and detect whether the pillar is on the left or the right side from the car, when the camera detects that the size of the pillar is bigger than 10000 the car should turn to the left or to the right, the turning angle is adjusted using the MPU6050 , the direction of the car’s movement is declared from detecting the lines on the map, when the color sensor detects 12 lines , the car stops moving.

## Lap counting mechanism

Problem statement:

The goal is to implement a lap counting mechanism for an autonomous vehicle (or RC car) that will accurately count the number of laps the vehicle completes on a designated track. The key requirement is that after completing exactly three laps, the vehicle must automatically stop.

1.	COLOR SENSOR-BASED DETECTION:
The first method involves utilizing a color sensor to detect specific markers placed on the track. This technique leverages the sensor's ability to identify distinct colors, allowing it to register a lap each time the sensor passes over a designated colored marker.
Principle: The color sensor is calibrated to recognize a specific color that is distinct from the track's surface. When the sensor detects this color, it triggers a signal to increment the lap count. 

Why Do we Not Recommend This Method?

Disadvantages:

Accuracy Issues: Despite our extensive efforts, the color sensor consistently struggled to provide accurate readings, particularly when detecting the color orange. The sensor's performance was significantly affected by ambient lighting conditions, even after attempts to mitigate this with a custom shield.
Programming Challenges: We encountered difficulties in programming the sensor to reliably identify the target color. These challenges were exacerbated by the high speed of the vehicle, which further impaired the sensor’s ability to accurately detect and register the color in real-time.
Unreliable Performance: Overall, the color sensor's inability to deliver consistent and precise measurements, especially under the dynamic conditions of our application, rendered it an unsuitable choice for our project. This unreliability was a critical factor in our decision to explore alternative lap counting methods.

2- Loop-Based Counting with Ultrasonic Sensor and IMU

The second method involves a software-based loop counter that increments the lap count each time a specific condition within the code is met. In our approach, the robot detects the completion of a lap using an ultrasonic sensor. When the sensor detects that the distance to a specific point or object is less than 50 cm, the robot recognizes this as an indicator to begin a new lap. The robot’s steering is controlled by an IMU (Inertial Measurement Unit), which ensures precise navigation as the robot turns.

Principle: The loop-based method in our project is implemented by continuously monitoring the distance using the ultrasonic sensor. When the sensor detects that the distance has decreased below 50 cm, the robot initiates a turning maneuver, guided by the IMU. This detection and turning process is managed within a loop structure in the code, specifically within a while loop that executes as long as the distance remains below 60 cm. Each time this condition is met, the program increments the lap count, allowing for accurate tracking of the robot’s laps.
