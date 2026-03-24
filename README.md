# Neuroprosthetics-robot-emg-control
End-to-end neuroprosthetics project combining cubic trajectory planning, obstacle avoidance, real PhantomX robot control, and EMG-driven human–machine interaction.

# 🧠 Neuroprosthetics Robot Control with EMG

Cubic trajectory planning, obstacle avoidance, real robot execution, and EMG-based gripper control for the **Neuroprosthetics** course (Master in Science in Neurotechnology, UPM).

---

## 📌 Overview

This project implements a full pipeline for robotic manipulation:

- 🟢 Cubic trajectory planning (CUBIC1 & CUBIC2)
- 🟢 Obstacle avoidance using intermediate waypoints
- 🟢 Execution on a real PhantomX robotic arm
- 🟢 EMG-based control of the gripper

---

## ⚙️ Technologies Used

- Arduino (C++)
- MATLAB
- EMG sensors
- PhantomX Robotic Arm

---

## 🧩 Project Structure
```
neuroprosthetics-robot-emg-control/
│
├── README.md
├── .gitignore
├── CITATION.cff
│
├── ARDUINO/
│   ├── GR01-D2a/
│   │   ├── GR01-D2a.ino
│   │   ├── poses.h
│   │   ├── robot.h
│   │   └── servos.h
│   │
│   ├── GR01-D2b/
│   │   ├── GR01-D2b.ino
│   │   ├── poses.h
│   │   ├── robot.h
│   │   └── servos.h
│
├── MATLAB/
│   └── D2.m
│
└── docs/
    └── report.pdf
```
---

## 🚀 Features

### 1. Cubic Trajectory Planning
Smooth motion generation between points using cubic polynomials.

### 2. Obstacle Avoidance
Collision-free trajectories via intermediate waypoint strategy.

### 3. Real Robot Execution
Implementation on PhantomX robotic arm hardware.

### 4. EMG-Based Control
Gripper control using muscle activity signals.

---

## 📊 Results

- Smooth and continuous trajectories achieved
- Successful obstacle avoidance
- Real-time robot execution validated
- Reliable EMG-based gripper control

---

## 📄 Documentation

Full report available in:

```
report.pdf
```

---

## 👨‍💻 Authors

- Daniel Llopis Conejo
- Matías Nevado García

Master in Science in Neurotechnology  
Universidad Politécnica de Madrid (UPM)

---

## 📜 License

This project is for academic purposes.
