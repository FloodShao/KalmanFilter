# Introduction
This project implements C++ to fuse Lidar and Radar sensor measurements. The data will be processed with Extended Kalman Filter and Uscented Kalman Filter.

**Sensor measurements:**
* A Lidar sensor measures the tracked object's position in cartesian-coordinates (x, y) 
* A Radar sensor measures a tracked object's position and relative velocity in polar coordinates (rho, phi, drho) (note: the velocity is along the line of sight) 

**Given that the tracked object is static, we can predict the position of the car, which includes a Lidar and a Radar, by predicting the object position and velocity.**
* We want to get the car position of the system in cartesian coordinates, the velocity magnitude, the yaw angle in radians, and yaw rate in radians per second (x, y, v, yaw, yawrate)
* We are assuming a Constant Turn and Velocity Model for this system

# Environment
The project is complied under macOS High Sierra 10.13.6.
Dependencies are: 
* cmake 3.14.1 (brew install cmake)
* GNU Make 3.81
* gcc 6.5.0

Install Xcode for IDE developing.

* eigen 3.3.7 For matrix calculation (brew install eigen)


