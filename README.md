# EYEMECH ε3.2 ESP32 Controller

[![EYEMECH ε3.2 Demo](https://img.youtube.com/vi/MeHLouL9ltw/0.jpg)](https://www.youtube.com/watch?v=MeHLouL9ltw "EYEMECH ε3.2 Demo - Click to Watch!")

## Overview

EYEMECH ε3.2 is an adaptation of Will Cogley's Eye Mechanism control code for the ESP32 microcontroller with PCA9685 servo controller. This project brings animatronic eye movements to life using MicroPython, allowing for both automatic and manual control of eye position and blinking.

## Features

- **Dual Control Modes**: Automatic random eye movement or manual joystick control
- **Realistic Eye Mechanics**: Coordinated movement of eyelids with eye position
- **Customizable**: Adjustable eye openness and movement limits
- **Low Latency**: Smooth transitions between positions
- **Easy Setup**: Complete step-by-step guide for hardware and software installation

## Hardware Requirements

- ESP32 WROOM Dev Module
- PCA9685 16-Channel PWM Servo Driver
- 6 Micro Servos:
  - Left/Right movement (1)
  - Up/Down movement (1)
  - Eyelids (4 - Top Left, Bottom Left, Top Right, Bottom Right)
- 1 PS4/PS5 Koystick module (for joystick control)
- 1 Trim potentiometer (for eye openness)
- Mode switch, Enable switch, and Blink button (Mode & Enable swichtes can be joined using a 2 way Switch)
- 5V-6V power supply for servos

## Wiring Diagram

```
ESP32                    PCA9685
-----------------        -----------------
GND         ------->     GND
3.3V/5V     ------->     VCC
GPIO21(SDA) ------->     SDA
GPIO22(SCL) ------->     SCL

ESP32 Inputs:
-----------------
GPIO14      <------- Mode Switch
GPIO13      <------- Enable Switch
GPIO15      <------- Blink Button
GPIO34      <------- Up/Down Joystick
GPIO32      <------- Left/Right Joystick
GPIO35      <------- Trim Potentiometer

PCA9685 Outputs:
-----------------
Channel 0   ------->     Left/Right Servo
Channel 1   ------->     Up/Down Servo
Channel 2   ------->     Top Left Eyelid Servo
Channel 3   ------->     Bottom Left Eyelid Servo
Channel 4   ------->     Top Right Eyelid Servo
Channel 5   ------->     Bottom Right Eyelid Servo
```

## Installation

### Step 1: MicroPython Setup

1. Download the latest MicroPython firmware for ESP32 from [micropython.org](https://micropython.org/download/esp32/)
2. Flash the firmware to your ESP32 using esptool or your preferred method

### Step 2: Thonny IDE Setup

1. Download and install [Thonny IDE](https://thonny.org/)
2. Configure Thonny:
   - Open Settings > Interpreter
   - Select "MicroPython ESP32" 
   - Select your ESP32's port
   - Click "OK"

### Step 3: Code Installation

1. Create a new project folder on your computer
2. Create two files: `main.py` and `pca9685.py`
3. Copy the provided code into these files
4. Upload both files to your ESP32 using Thonny's "Save as... > MicroPython device" option

## How It Works

The system operates in three modes:

1. **Calibration Mode**: When the mode switch is held, all servos move to their initial positions for assembly and calibration.

2. **Automatic Mode**: When the enable switch is off, the eyes will:
   - Randomly look around
   - Blink at random intervals
   - Change expressions autonomously

3. **Controller Mode**: When the enable switch is on, you can:
   - Control eye position using the joystick potentiometers
   - Adjust eye openness with the trim potentiometer
   - Trigger blinking with the blink button

## Code Structure

- **PCA9685 Driver**: Handles communication with the servo controller
- **Servo Mapping**: Defines channel assignments and movement limits
- **Scaling Functions**: Converts potentiometer readings to appropriate servo angles
- **Eyelid Control**: Coordinates eyelid movement with eye position
- **Mode Handling**: Manages switching between operation modes

## Configuration

You can adjust the following parameters in the code:

- `servo_limits`: Set minimum and maximum angles for each servo
- `max_speed`: Limit the maximum speed of eye movement
- `deadzone`: Adjust the joystick center deadzone
- Potentiometer center values: Calibrate the center position of joysticks

## Acknowledgments

Based on Will Cogley's Eye Mechanism control code:
- [Instructables Guide](https://www.instructables.com/Animatronic-Eye-Mechanism/)
- [Patreon](https://www.patreon.com/c/Will_Cogley/posts)
- [Documentation](https://willcogley.notion.site/EyeMech-3-2-1af24779b64d80b19edfdd795d4b90e5)
- [3D Models](https://makerworld.com/es/models/1184807-animatronic-eye-mechanism-e3-2)

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request
