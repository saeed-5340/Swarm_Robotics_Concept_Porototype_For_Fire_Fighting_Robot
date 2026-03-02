# Swarm_Robotics_Concept_Porototype_For_Fire_Fighting_Robot

[![C](https://img.shields.io/badge/language-C-72.1%25-blue)](https://github.com/saeed-5340/Swarm_Robotics_Concept_Porototype_For_Fire_Fighting_Robot)
[![C++](https://img.shields.io/badge/language-C++-27.9%25-orange)](https://github.com/rashed-23/Swarm_Robots_and_Fire_Fighting)

This repository contains the complete codebase and documentation for an **AI‑Enabled Swarm Robotics system** that autonomously detects and extinguishes fires using a master‑slave robot architecture.

## 📁 Repository Contents

- `Master_Bot/` – Main code for the master robot (fire detection, sensor fusion, ESP‑NOW transmission)
- `Slave_Bot/` – Main code for the slave robot (navigation, pump control, ESP‑NOW reception)
- `CameraWebServer/` – ESP32‑CAM streaming and image capture
- `Sending_Code_esp32_to_esp32/` & `Reciving_Code_esp32_to_esp32/` – ESP‑NOW communication examples
- `MeasureDistance_...` folders – HC‑SR04 ultrasonic sensor tests
- `Test_MPU6050/` – MPU6050 IMU test
- `L298N_Motor_Control_Test/` – L298N motor driver test
- `Servo_Test_SG90/` – Servo motor test
- `DesignandImplementationofAI-EnabledSwarmRobotsforAutonomousFireDetectionandSuppression.pdf` – Full project report with circuit diagrams, algorithms, and system details

## 🛠️ Hardware Overview

- **Master Bot:** ESP32, ESP32‑CAM, HC‑SR04, MPU6050, KY‑026 flame sensor, servo motor, L298N, DC motors
- **Slave Bot:** ESP32, HC‑SR04, MPU6050, L298N, DC motors, water pump

## 🚀 Quick Start

1. Clone the repository.  
2. Install ESP32 board support in Arduino IDE (add `https://dl.espressif.com/dl/package_esp32_index.json` to Board Manager URLs).  
3. Install required libraries: `NewPing`, `MPU6050`, `ESP32Servo`.  
4. Test individual components using the sketches in the respective folders.  
5. Upload `Master_Bot` and `Slave_Bot` codes and configure ESP‑NOW communication.

## 📖 Full Documentation

All project details, including circuit diagrams, sensor fusion logic, PID control, coordinate transformations, and system limitations, are available in the **[PDF report](DesignandImplementationofAI-EnabledSwarmRobotsforAutonomousFireDetectionandSuppression.pdf)** .

## 👥 Contributors

- Saeed Ahamed Mridha (2020338012)
- Md. Rashedul Islam (2020338023)
- Md. Rimon Hossain (2020338007)
- Md. Sohel Rana (2020338034)
- Md. Mehedi Hasan (2020338037)
- Shuvo Ghose Shishir (2019338045)
- Kowshic Mazumder Niloy (2019338079)

**Supervisor:** Md. Kabir Hasan, Lecturer, Dept. of EEE, SUST

---

*Project developed at the Microprocessor and Interfacing Lab, Shahjalal University of Science and Technology.*
