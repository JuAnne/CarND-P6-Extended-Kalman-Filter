# CarND-P6-Extended-Kalman-Filter
Udacity Self-Driving Car Nanodegree - Extended Kalman Filter Implementation

## Overview
This project consists of implementing an [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) with C++. A simulator provided by Udacity ([it could be downloaded here](https://github.com/udacity/self-driving-car-sim/releases)) generates noisy RADAR and LIDAR measurements of the position and velocity of an object, and the Extended Kalman Filter[EKF] must fusion those measurements to predict the position of the object. The communication between the simulator and the EKF is done using the [uWebSockets](https://github.com/uNetworking/uWebSockets) implementation on the EKF side. To get this project started, Udacity provides a seed project that could be found [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).

## Prerequisites
The project has the following dependencies (from Udacity's seed project):

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.

This particular implementation is done on Linux OS. In order to install the necessary libraries, use the install-ubuntu.sh.

## Compiling and executing the project
These are the suggested steps:

- Clone the repo and cd to it on a Terminal.
- Create the build directory: `mkdir build`
- `cd build`
- `cmake ..`
- `make`: This will create an executable `ExtendedKF`
