# RobOlympics 2024-2025

**Competition Robotics Projects**

Two robots built for the RobOlympics competition: a Bluetooth-controlled soccer robot and an autonomous line-following robot.

---

## Competition Results

- **Soccer Robot**: 1st Place
- **Line Following Robot**: Close 3rd (lost by 3 seconds due to sensor calibration)

---

## Projects

### 1. Soccer Robot

Bluetooth-controlled robot with 8-directional movement and electronic braking system.

![Soccer Robot](Soccer/robot%20in%20action/soccer_robot.jpg)
*Soccer robot in competition*

**Features:**
- 8-directional control (forward, back, left, right, and diagonals)
- 11-speed levels (0-9, q) for precise control
- Electronic braking system for faster stopping
- Serial command interface via Bluetooth

**Hardware:**
- L298N Motor Driver
- 2× DC Motors
- Bluetooth module (HC-05/HC-06)
- Arduino UNO

**Controls:**
- **Movement**: F (forward), B (back), L (left), R (right)
- **Diagonals**: G (forward-left), I (forward-right), H (back-left), J (back-right)
- **Speed**: 0-9, q (0 = slowest, q = fastest)
- **Stop**: S (with electronic brake)

**Code:**
- `soccer.ino` - Main control code with electronic braking

---

### 2. Line Following Robot

Autonomous robot using PID control for smooth line tracking.

![Line Following Robot](Line%20Following/robot%20in%20action/line_follower.jpg)
*Line following robot during competition*

**Features:**
- PID (Proportional) control for smooth corrections
- Automatic line search and recovery
- Dual IR sensor system
- Competition-optimized speed tuning

**Hardware:**
- L298N Motor Driver
- 2× DC Motors
- 2× IR Line Sensors (digital)
- Arduino UNO

**Algorithm:**
- Uses proportional control (Kp = 10.0) to calculate motor speed adjustments
- Differential steering for smooth curve handling
- Spin-to-search recovery when line is lost

**Code:**
- `Line Following/final_code/` - PID competition version
- `Line Following/testing_components/` - Initial testing/calibration code

**Note:** Robot performed well but lost by 3 seconds in finals due to sensor positioning relative to track lighting (lesson learned: always test under competition lighting conditions).

---

## Project Structure

```
ROBOLYMPICS/
├── Line Following/
│   ├── final_code/              # PID competition version
│   ├── robot in action/         # Photos/videos
│   └── testing_components/      # Testing/calibration code
├── Soccer/
│   ├── robot in action/         # Photos/videos
│   ├── soccer.ino              # Main control code
│   └── README.md               # This file
```

---

## Technical Highlights

### Soccer Robot
- Implemented electronic braking system by briefly energizing opposing motor directions
- Variable speed control with 11 calibrated speed levels
- Diagonal movement for precise ball control

### Line Following Robot
- PID control for smoother line tracking than bang-bang methods
- Automatic recovery system when line is lost
- Optimized speed/accuracy tradeoff for competition performance

---

## Lessons Learned

1. **Test under actual conditions**: Sensor calibration for competition lighting is critical
2. **Iterate rapidly**: Started with simple bang-bang control, upgraded to PID for better performance
3. **Modular design**: Separate testing code helped verify hardware before deploying competition code
4. **Electronic braking**: Small optimizations (like active braking) provide competitive advantages

---

## Competition

**RobOlympics 2024-2025**
- High school robotics competition
- Multiple event categories
- 1st place in Soccer category
- Strong performance in Line Following (lost by 3 seconds in final round)

---

## Hardware Specifications

### Motors & Drivers
- **Motor Driver**: L298N H-Bridge
- **Motors**: Standard DC gear motors
- **Power**: 9V battery (motors), separate Arduino power

### Sensors (Line Following)
- **Type**: IR reflectance sensors (digital output)
- **Quantity**: 2 (left and right)
- **Logic**: 0 = black (line), 1 = white (background)

### Communication (Soccer)
- **Protocol**: Bluetooth Serial (9600 baud)
- **Module**: HC-05 or HC-06
- **Range**: ~20 meters (surprisingly it worked even when we were outside of school building?!)

---

## Development Process

1. **Design**: Planned electrical systems and control algorithms
2. **Prototype**: Built initial versions with breadboards
3. **Test**: Used simple control code to verify hardware
4. **Optimize**: Implemented PID and electronic braking
5. **Compete**: Deployed competition-ready code

---

Built with motors, sensors, and a lot of debugging (and I mean A LOT).