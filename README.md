# 4-DOF Robotic Arm with Custom Arduino Shield and IK Controller

This project provides a complete hardware and software solution for controlling a 4-DOF (Degrees of Freedom) acrylic robotic arm. It includes a custom Arduino shield, stable firmware, and a Python-based inverse kinematics (IK) controller for high-level Cartesian control.

https://github.com/user-attachments/assets/bff3e96c-0358-43f1-9e7e-ef8ffabbe7ce

## ✨ Features

-   **Custom Hardware:** A custom-designed Arduino Uno shield for robust servo connections and power management.
-   **Stable Firmware:** Jitter-free servo control using direct hardware timer manipulation instead of the standard Servo library.
-   **High-Speed Communication:** A fast, token-based serial protocol running at **250000 baud**.
-   **Inverse Kinematics (IK) Controller:** A Python script that allows you to control the arm by specifying target (X, Y, Z) coordinates, with a real-time 3D visualization.
-   **Modular Design:** The project is clearly organized into `hardware`, `firmware`, and `control_software` directories.

## 📂 Repository Structure

```
Robotic-Arm/
├── control_software/     # Python IK controller and visualization
│   ├── arm_ik_controller.py
│   └── README.md
├── docs/                 # Diagrams and documentation
│   └── block_diagram.jpg
├── firmware/             # Arduino PlatformIO project
│   ├── platformio.ini
│   └── src/
│       └── main.cpp
├── hardware/             # KiCad design files for the shield
│   ├── kicad_files/
│   └── README.md
└── README.md             # Main project README
```

## ⚙️ Getting Started

### 1. Hardware Setup

1.  **Assemble the Shield:** The KiCad project files are in the `hardware/kicad_files` directory. Use them to manufacture and assemble the Arduino shield.
<p align="center">
  <img src="https://github.com/user-attachments/assets/94d61672-353b-4d17-881a-5043d9a84109" alt="Assembled Robotic Arm Shield" width="350" />
</p>

2.  **Connect Components:**
    -   Mount the shield onto an Arduino Uno.
    -   Connect the four servo motors (Base, Shoulder, Elbow, Wrist) to the corresponding headers on the shield.
    -   Connect an appropriate 5V power supply for the servos.

### 2. Firmware Setup

The firmware uses **PlatformIO**, which handles all dependencies automatically.

1.  **Open in VS Code:** Open the `firmware` directory in Visual Studio Code with the PlatformIO extension installed.
2.  **Build & Upload:** Connect the Arduino Uno via USB and upload the firmware using the PlatformIO "Upload" task.

### 3. Control Software Setup

The Python script allows you to control the arm using inverse kinematics.

1.  **Install Dependencies:**
    ```sh
    pip install pyserial matplotlib
    ```
2.  **Run the Controller:**
    -   Navigate to the `control_software` directory.
    -   Run the script:
        ```sh
        python arm_ik_controller.py
        ```
    -   The script will prompt you to select the correct serial port for the Arduino.
    -   Choose a control mode (Interactive, Joint Sweep, etc.) and start sending commands!

## 🕹️ Serial Command Reference

For direct control, you can use a serial monitor (e.g., Arduino IDE's) at **250000 baud**.

| Command | Description                                         | Example                     |
| :------ | :-------------------------------------------------- | :-------------------------- |
| `E`     | **E**nable power to all servos.                     | `E`                         |
| `D`     | **D**isable power to all servos.                    | `D`                         |
| `H`     | Move the arm to the predefined **H**ome position.   | `H`                         |
| `P`     | Set servo **P**ositions (angles 0-180).             | `P 90 45 120 180`           |

## Final Results:
<img src="https://github.com/user-attachments/assets/8b99a545-8eb2-4781-97c2-95de9f968b7a" alt="IMG-20251025-WA0017" width="400">
<img src="https://github.com/user-attachments/assets/f99d57ff-65df-4117-8323-c3f7e8e6f8e9" alt="IMG-20251025-WA0020" width="400">

## 📜 License

This project is licensed under the **GNU General Public License (GPL) v3.0**.
