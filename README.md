# ApogeeEstimation
This project implements a high-precision recovery system for model rockets, utilizing the STM32 Nucleo F446RE microcontroller. The system ensures safe and efficient parachute deployment by monitoring altitude, acceleration, and GPS coordinates in real time, with sensor fusion using a Kalman filter to improve accuracy.

Features:
1. Altitude-Based Parachute Deployment – Uses a DPS310 barometric pressure sensor to determine the apogee and trigger the parachute release.
2. GPS Tracking for Recovery – Integrates a NEO-6M GPS module to track the rocket’s landing coordinates.
3. Acceleration-Based Event Detection – Employs an ADXL345 accelerometer to detect high-G events like burnout and deployment.
4. Sensor Fusion with Kalman Filter – Combines altitude, acceleration, and GPS data to filter out noise and provide precise state estimation for event detection.
5. Failsafe Mechanism – Implements multiple checks (altitude, acceleration, and time-based) to ensure reliable parachute deployment even in case of sensor failures.
6. FreeRTOS for Task Management – Ensures real-time processing of sensor data and event handling.
   
Repository Structure:
/Src – Contains the core source code, including sensor drivers, data processing, and parachute deployment logic.
/Inc – Header files defining functions and data structures.
/Docs – Includes wiring schematics, system flowcharts, and project documentation.

