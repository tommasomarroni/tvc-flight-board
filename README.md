# Model rocket flight board (beta)
*[abandoned project]*

Compact (beta) flight board for an actively controlled (thrust vectoring) model rocket.

## Running

Upload code on board with:
```
arduino --upload main.ino --port <port>
```

## Hardware and Software

**Hardware:**
- Up to 4 MG90S servos
- Arduino Nano v3.0
- MPU 6050 6-DOF IMU
- BMP 280 pressure sensor
- MicroSD card adapter
- Other: resistors, buzzer, led, power switch

**This code uses:**
- 29782 bytes of program space: 96% of Arduino Nano v3.0 program space limit of 30720 bytes;
- 1403 byte of dynamic memory: 68% of Arduino Nano v3.0 dynamic memory limit.
in NON-DEBUG mode.

**This code implements:**
- Attitude, temperature, pressure, altitude measuring
- Data log into MicroSD
- IMU Error-calibration procedure
- S. Madgwick filtering to get quaternions
- Kalman filtering to filter out euler angles (after conversion)
- Low pass filtering when needed
- Gravity compensation in acceleration components
- PID based attitude control
- Automatic activation, deactivation of TVC and parachute ejection

**Future developments:**
- Use 9DOF IMU;
- Use a more powerful microcontroller unit: Arduino Nano 33 BLE Sense, Teensy 4.0, STM32Fx.

**This is a beta/basic version, strongly limited by the original Arduino Nano v3.0 program space limit.**

## References

- [TVC Model Rocket](https://www.tommasomarroni.com/random-posts/)
