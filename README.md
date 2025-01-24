# SmartNav

This project implements a PID-controlled robot that uses an ultrasonic sensor to detect obstacles and adjust its movements accordingly. The robot is equipped with two DC motors driven by an L298N motor driver and an HC-SR04 ultrasonic sensor for distance measurement.

## Features
- **PID Control**: Ensures smooth and accurate distance maintenance from obstacles.
- **Obstacle Avoidance**: Detects objects within a specified range and adjusts movement direction.
- **Bidirectional Movement**: Moves forward, backward, and can rotate left or right.
- **Real-Time Data**: Distance and PID output are logged via the Serial Monitor for debugging and analysis.

## Hardware Requirements
1. **Arduino Board** (e.g., Arduino Uno, Mega, or similar).
2. **HC-SR04 Ultrasonic Sensor** for distance measurement.
3. **L298N Motor Driver Module** to control DC motors.
4. **2 DC Motors** for driving the robot.
5. **Power Source** (e.g., batteries).
6. Jumper wires and a breadboard for connections.

## Wiring Diagram
### Motor Driver
- **Motor A (Left)**
  - ENA: Pin 22
  - IN1: Pin 19
  - IN2: Pin 18

- **Motor B (Right)**
  - ENB: Pin 23
  - IN3: Pin 17
  - IN4: Pin 16

### Ultrasonic Sensor
- TRIGGER_PIN: Pin 33
- ECHO_PIN: Pin 32

## Software Setup
### Dependencies
This project uses the following Arduino libraries:
- `Wire.h`: For I2C communication (if needed).
- `NewPing.h`: For ultrasonic sensor operations.
- `L298N.h`: For controlling the L298N motor driver.

Install these libraries using the Arduino Library Manager or download them from their respective sources.

### Installation
1. Clone this repository or copy the code into an Arduino IDE sketch.
2. Connect the hardware as per the wiring diagram.
3. Upload the sketch to your Arduino board.
4. Open the Serial Monitor to view real-time data.

## Configuration
### PID Constants
Modify the following constants in the code to tune the PID controller:
- `kp`: Proportional gain.
- `ki`: Integral gain.
- `kd`: Derivative gain.

### Set Point
Set the desired distance from obstacles by adjusting the `setPoint` variable (in cm).

## How It Works
1. The ultrasonic sensor measures the distance to the nearest obstacle.
2. The PID controller calculates the error between the set point and the measured distance.
3. The robot adjusts its speed and direction based on the PID output to maintain the desired distance.
4. If no obstacles are detected, the robot moves forward.
5. If an object is detected within 10 cm, the robot reverses to avoid the obstacle.

## Serial Monitor Output
The Serial Monitor displays:
- Measured distance (in cm).
- PID output values.
- Current robot movement direction.

## Troubleshooting
- **No movement**: Check motor wiring and ensure the motor driver is powered correctly.
- **Incorrect distance readings**: Verify ultrasonic sensor connections and orientation.
- **Erratic movements**: Adjust the PID constants for smoother control.

## License
This project is open-source and available under the MIT License.

---

Happy Building! 🚀
