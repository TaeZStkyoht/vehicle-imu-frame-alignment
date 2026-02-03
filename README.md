# imu-mounting-calibration

GNSS-free online calibration of IMU mounting angles for ground vehicles, based on a physics-based optimization approach.

This project implements the mounting angle estimation algorithm described in:

> **Senofieni, R., Corno, M., Strada, S. C., Savaresi, S. M., & Fredella, S. (2023).**
> *GNSS-free Online Calibration of Inertial Measurement Units in Road Vehicles*
> IFAC-PapersOnLine, 56(3), 1027-1032.
> DOI: 10.1016/j.ifacol.2023.12.009

The implementation is written in **C++** and is designed for **embedded and automotive-grade IMU systems**.

---

## Overview

In real-world vehicle installations, an IMU is rarely mounted perfectly aligned with the vehicle reference frame.  
This misalignment introduces systematic errors in acceleration- and orientation-based algorithms.

This project estimates the **IMU-to-vehicle mounting rotation** using only:

- Accelerometer measurements
- Gyroscope measurements

No GNSS, magnetometer, wheel speed, or external reference sensors are required.

The method is **online-capable** and relies on vehicle motion constraints and gravity consistency.

---

## Assumptions and Scope

The algorithm is intended for:

- Ground vehicles (cars, trucks, wheeled vehicles)
- Motion constrained to the Earth surface
- Moderate dynamics (normal driving conditions)

It is **not suitable** for:
- Aerial vehicles (UAVs, rockets)
- Free-fall or ballistic motion
- Marine or underwater vehicles without additional constraints

---

## Input Data Requirements

| Signal | Unit |
|------|------|
| Accelerometer | g |
| Gyroscope | rad/s |

---

## Output

The algorithm estimates:

- A 3D rotation (IMU → vehicle frame)
- Represented internally as a rotation matrix or quaternion

This rotation can be used to transform raw IMU measurements into the vehicle reference frame.

---

## Status

- [x] Core algorithm implemented
- [x] Run-time offline evaluation with recorded datasets
- [x] Compile-time offline evaluation with recorded datasets
- [x] Online convergence tuning
- [x] Well-structured code
- [ ] Parameter refinement
- [ ] Extended documentation

---

## License

This project is intended for research and development purposes.  
Refer to the original paper for academic use and citation requirements.

