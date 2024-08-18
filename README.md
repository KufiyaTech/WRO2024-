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

### How Does Our Robot Think

To ensure our vehicle can successfully complete the tasks required in both rounds of the competition, we have utilized a variety of sensors, each with a specific function, as detailed in the Power and Sense Management section of this README.

Our robot is powered by an Arduino Mega microcontroller and programmed in C++. The core logic of the vehicle relies on a PD (Proportional-Derivative) controller to navigate within the designated safe zone. The vehicle is equipped with three ultrasonic sensors and an IMU (Inertial Measurement Unit) sensor, which together manage the steering system effectively.

Additionally, a front-facing ultrasonic sensor is employed to detect U-turns. This combination of sensors and control algorithms forms the backbone of our robot's decision-making process, allowing it to navigate the course autonomously and efficiently.
