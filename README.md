# 🤖 README – B-Bot Self-Balancing & Vision-Based Robot

## 📂 Project Structure

```
B-Bot/
│
├── computer_vision/
│   ├── vision.py
│   └── requirements.txt
│
├── fuzzyControlArduino/
│   ├── fuzzyControlArduino.ino
│   ├── MPU6050_tockn.cpp
│   ├── MPU6050_tockn.h
│   ├── testingMotots.ino
│   └── Trials/
│       ├── pid.ino
│       ├── thetaonly.ino
│       └── trial2.ino
│
├── matlab/
│   ├── b_bot.fis
│   └── Self_balancing_robot.slx
│
├── SOLIDWORKS/
│   ├── Robot Final (1).STL
│   └── Robot2.SLDPRT
│
└── README.md
```

## ⚙️ Main Functionalities Explained

| Component/Folder                     | Purpose and Functionality |
|-------------------------------------|----------------------------|
| `computer_vision/vision.py`         | Raspberry Pi-based computer vision script that uses YOLOv8 to detect humans. If a human is detected, the image is compared to reference faces using SSIM. LEDs provide real-time feedback. |
| `fuzzyControlArduino.ino`           | Arduino sketch implementing a fuzzy logic controller that keeps the robot balanced using tilt (theta) and angular velocity (dTheta) from the MPU6050 sensor. |
| `Trials/`                            | Experimental control codes: `pid.ino` for PID controller, `thetaonly.ino` for using only tilt angle, `trial2.ino` for hybrid attempts. |
| `testingMotots.ino`                 | Standalone motor testing script to check PWM behavior and motor direction. |
| `MPU6050_tockn.cpp/h`               | Custom library for interfacing with the MPU6050 IMU module for tilt and angular velocity. |
| `matlab/b_bot.fis`                  | MATLAB Fuzzy Inference System (FIS) file with the same logic used in Arduino; useful for visual tuning and simulations. |
| `matlab/Self_balancing_robot.slx`   | Full Simulink simulation model with multibody setup to test the control system virtually. |
| `SOLIDWORKS/Robot Final (1).STL`    | Exported 3D printable chassis design for fabrication. |
| `SOLIDWORKS/Robot2.SLDPRT`          | Source CAD file for modifying robot design in SolidWorks. |

---

## 🛠️ Requirements

### For Raspberry Pi:
Install the required packages:
```bash
cd computer_vision
pip install -r requirements.txt
```

Key packages:
- `ultralytics` (YOLOv8)
- `opencv-python`
- `picamera2`
- `scikit-image`
- `RPi.GPIO`

System-level dependencies:
```bash
sudo apt install -y python3-picamera2 python3-rpi.gpio
```

---

## 🚀 How to Run

### Computer Vision (on Raspberry Pi):
```bash
cd computer_vision
python3 vision.py
```

### Arduino:
- Open `fuzzyControlArduino.ino` or any file from `Trials/` in Arduino IDE
- Select your board and COM port
- Upload to your Arduino

### MATLAB/Simulink:
- Open `b_bot.fis` in the Fuzzy Logic Designer
- Simulate `Self_balancing_robot.slx` to validate or tune the logic

---

## 🎥 Demo Video

[![Watch the Full Video Demo](https://img.youtube.com/vi/OxzO8reckg4/0.jpg)](https://www.youtube.com/watch?v=OxzO8reckg4)

---

> Created by **Mohamed El Nakouzi, Rached El Bitar, and Daniel Nassar** — combining fuzzy logic, computer vision, and mechanical design in a mechatronic system.
