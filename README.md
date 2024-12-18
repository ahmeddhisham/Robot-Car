# Enhanced Robot Car Controller

This project is an improved version of a robot car controller designed for obstacle detection and motion control. It uses three ultrasonic sensors for distance measurement and a TB6612FNG motor driver to control the motors. The robot is capable of adjusting its movement based on real-time sensor data, performing obstacle avoidance maneuvers, and executing recovery actions when stuck.

## Features

- **Obstacle Detection**: Utilizes three ultrasonic sensors (left, center, right) to detect obstacles and measure distances in the robot's environment.
- **Motor Control**: Uses a TB6612FNG motor driver to control the speed and direction of two DC motors.
- **Obstacle Avoidance**: Adjusts the robot’s speed and direction to avoid obstacles detected by the ultrasonic sensors.
- **Recovery Maneuver**: If the robot detects obstacles in all directions and gets stuck, it will perform a recovery maneuver by reversing and spinning to free itself.
- **PWM Speed Control**: Motor speed is controlled using Pulse Width Modulation (PWM) for smooth movement.
- **Diagnostic Logging**: Outputs sensor readings and motor speeds to the Serial Monitor for easy debugging.

## Hardware Requirements

- **Arduino Board** (e.g., Arduino Uno or similar)
- **TB6612FNG Motor Driver**
- **3 Ultrasonic Sensors** (HC-SR04 or compatible)
- **2 DC Motors** (for left and right wheels)
- **Motor Wheels and Chassis** (for assembly)
- **Power Supply** (battery or USB power for the Arduino)

## Circuit Diagram

The following pins are used for the motor and sensor connections:

- **Motors:**
  - Left Motor Forward: Pin 9
  - Left Motor Reverse: Pin 8
  - Right Motor Forward: Pin 7
  - Right Motor Reverse: Pin 6
  - Left Motor PWM: Pin 5
  - Right Motor PWM: Pin 10

- **Ultrasonic Sensors:**
  - Left Ultrasonic Trigger: Pin 2
  - Left Ultrasonic Echo: Pin 3
  - Center Ultrasonic Trigger: Pin 4
  - Center Ultrasonic Echo: Pin 11
  - Right Ultrasonic Trigger: Pin 12
  - Right Ultrasonic Echo: Pin 13

## Code Explanation

1. **Motor Control**: The code controls the motor driver using the `digitalWrite` function to set the direction of the motors (forward or reverse), and `analogWrite` to adjust the motor speeds with PWM.
2. **Sensor Scanning**: The robot performs scans using three ultrasonic sensors to measure the distances to obstacles. The robot's movement is adjusted based on these readings.
3. **Obstacle Avoidance**: The robot changes its direction and speed based on the sensor data to avoid obstacles. If obstacles are detected on both sides, it will adjust the motor speeds to avoid collisions.
4. **Recovery Maneuver**: If all sensors detect an obstacle, the robot will reverse and spin to free itself, allowing it to continue moving.
5. **Diagnostics**: Sensor readings and motor speeds are logged to the Serial Monitor for troubleshooting and debugging.

## Usage

Once the code is uploaded to the Arduino, the robot will start scanning the environment using the ultrasonic sensors. The robot will adjust its speed and direction to avoid obstacles and perform recovery maneuvers if it gets stuck. The sensor readings and motor speeds are printed to the Serial Monitor for diagnostic purposes.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- This project is built on the Arduino platform with the TB6612FNG motor driver and HC-SR04 ultrasonic sensors.
- Special thanks to the open-source community for their contributions to robotics and Arduino projects.

