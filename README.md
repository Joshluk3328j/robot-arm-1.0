# 5-DOF Robotic Arm

A 5 Degrees of Freedom (DOF) robotic arm controlled by an Arduino microcontroller. This project features precise servo motor control with cosine-eased smooth transitions, motion recording and playback stored in EEPROM, and intuitive button-based operation with buzzer feedback. Designed for automation tasks, educational purposes, or hobbyist robotics.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Circuit Diagram](#circuit-diagram)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Overview
The 5-DOF robotic arm is a compact, versatile platform for learning robotics and automation. It uses five servo motors to achieve flexible movements, controlled via potentiometers or pre-recorded motion sequences. The physical design is optimized for stability and ease of assembly, as shown in the 3D rendering below.

![3D Rendering of 5-DOF Robotic Arm](https://github.com/yourusername/5-dof-robotic-arm/raw/main/docs/images/3d_rendering.png)

## Features
- **5-DOF Control**: Independent control of base, shoulder, elbow, wrist, and gripper via five servo motors.
- **Smooth Transitions**: Cosine easing for fluid 2-second servo movements, eliminating jerky motion.
- **Motion Recording and Playback**: Stores up to 100 motion frames (5 servo angles each) in EEPROM, with 1-second pauses between motions and a 3-second delay before returning to home position.
- **Dual Modes**:
  - **Control Mode**: Manual positioning using potentiometers.
  - **Play Mode**: Automatic playback of recorded sequences.
- **Button Controls**:
  - Button 1: Single press to save motion (Control Mode); double press to switch modes.
  - Button 2: Single press to pause/resume (Play Mode); double press to delete last motion (Control Mode) or clear all motions (Play Mode).
- **Audio Feedback**: Buzzer emits distinct beeps for button presses and motion completion.
- **EEPROM Storage**: Persistent motion storage in non-volatile memory.
- **Debounced Inputs**: Reliable button handling with 50ms debounce and 600ms double-click detection.

## Hardware Requirements
- Arduino Uno (or compatible microcontroller)
- 5x Servo Motors (e.g., SG90 or MG996R)
- 5x Potentiometers (10kΩ recommended)
- 2x Push Buttons (with internal pull-up resistors)
- 1x Piezo Buzzer
- External Power Supply (5V-6V, sufficient for servos)
- Breadboard or custom PCB
- Jumper wires
- USB cable for programming and debugging

### Pin Configuration
| Component       | Arduino Pins       |
|-----------------|--------------------|
| Servo Motors    | D3, D5, D6, D9, D10 |
| Potentiometers  | A1, A2, A3, A4, A5 |
| Button 1        | D2                 |
| Button 2        | D4                 |
| Buzzer          | D7                 |

## Circuit Diagram
The circuit connects the Arduino to five servo motors, five potentiometers, two buttons, and a buzzer. Servo motors are powered externally to avoid overloading the Arduino, while buttons use internal pull-up resistors for simplified wiring. The schematic below illustrates the connections.

![Circuit Schematic](https://github.com/yourusername/5-dof-robotic-arm/raw/main/docs/images/circuit_schematic.png)

## Software Requirements
- Arduino IDE (version 2.0 or higher)
- Libraries (included with Arduino IDE):
  - `Servo.h`
  - `EEPROM.h`
- Serial Monitor (9600 baud) for debugging

## Installation
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/5-dof-robotic-arm.git
   ```
2. **Assemble Hardware**:
   - Connect servo motors to PWM pins (D3, D5, D6, D9, D10).
   - Wire potentiometers to analog pins (A1-A5).
   - Attach buttons to digital pins (D2, D4) with internal pull-up resistors.
   - Connect the buzzer to D7.
   - Use an external power supply for servos.
3. **Install Arduino IDE**:
   - Download from [arduino.cc](https://www.arduino.cc/en/software).
4. **Upload Code**:
   - Open `5DOF_Robotic_Arm.ino` in the Arduino IDE.
   - Connect the Arduino via USB.
   - Upload the code.

## Usage
1. **Power On**:
   - Connect the Arduino and external power supply.
   - The arm initializes by moving smoothly to home angles (90° for all servos) and then to positions set by potentiometers.
2. **Control Mode**:
   - Adjust potentiometers to manually position the arm.
   - **Button 1**:
     - Single press: Save current servo angles as a motion frame (up to 100 frames).
     - Double press: Switch to Play Mode.
   - **Button 2**:
     - Double press: Delete the last saved motion.
   - Buzzer beeps confirm actions (short for single press, longer for double press).
3. **Play Mode**:
   - Plays recorded motions with 1-second pauses between each motion and a 3-second pause before returning to home.
   - **Button 1**:
     - Double press: Switch to Control Mode with smooth transition to potentiometer positions.
   - **Button 2**:
     - Single press: Pause/resume playback.
     - Double press: Clear all saved motions.
   - Buzzer beeps after each motion completes.
4. **Debugging**:
   - Open Serial Monitor (9600 baud) to view mode changes, motion storage, and playback status.
   - Example messages: "Motion stored. Total motions: X", "Switched to PLAY MODE", "Paused playback."
5. **Customization**:
   - Edit `homeAngles[]` in the code to change the default home position.
   - Adjust `transitionDuration` (default 2000ms) for faster/slower transitions.
   - Modify `motionDelay` (1000ms) or `endDelay` (3000ms) for playback timing.

## Contributing
Contributions are welcome! To contribute:
1. Fork the repository.
2. Create a branch: `git checkout -b feature/your-feature`.
3. Commit changes: `git commit -m "Add your feature"`.
4. Push: `git push origin feature/your-feature`.
5. Open a Pull Request.

Please include clear comments in code and follow the existing style.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.