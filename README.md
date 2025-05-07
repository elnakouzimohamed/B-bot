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
│   └── MPU6050_tockn.h
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

| Component/Folder                | Purpose and Functionality |
|--------------------------------|----------------------------|
| `computer_vision/vision.py`    | Raspberry Pi-based computer vision script that uses YOLOv8 to detect humans. If a human is detected, the image is compared to reference faces using SSIM. LEDs and a buzzer provide real-time feedback. |
| `fuzzyControlArduino.ino`      | Arduino sketch implementing a fuzzy logic controller that keeps the robot balanced using tilt (theta) and angular velocity (dTheta) from the MPU6050 sensor. |
| `MPU6050_tockn.cpp/h`          | Custom library for interfacing with the MPU6050 IMU. |
| `matlab/b_bot.fis`             | MATLAB Fuzzy Inference System (FIS) file containing the same fuzzy logic rules used in the Arduino implementation, useful for simulation or tuning. |
| `matlab/Self_balancing_robot.slx` | Simulink model of the control system, useful for simulation or future expansion. |
| `SOLIDWORKS/Robot Final (1).STL` | 3D printable version of the robot’s chassis. |
| `SOLIDWORKS/Robot2.SLDPRT`     | Editable SolidWorks CAD file of the robot design. |

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

For Raspberry Pi, also install system dependencies:
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
- Open `fuzzyControlArduino.ino` in Arduino IDE
- Upload to your board after selecting the correct COM port

### MATLAB/Simulink:
- Open `b_bot.fis` in Fuzzy Logic Designer
- Simulate `Self_balancing_robot.slx` if desired

---

## 🎥 Demo Video

### ▶️ Embedded YouTube Video

> Replace `YOUR_VIDEO_ID` with the actual ID after uploading.

```html
<iframe width="560" height="315" src="https://youtu.be/OxzO8reckg4" frameborder="0" allowfullscreen></iframe>
```

### 🔗 Direct YouTube Link

[Watch the Full Demo on YouTube](https://www.youtube.com/watch?v=YOUR_VIDEO_ID)

---

> Created by **Khaled Bahri** and team – built with ❤️ combining fuzzy logic, computer vision, and mechanical design.
