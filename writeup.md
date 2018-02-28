# Model Predictive Control Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The goal of this project is to implement an Model Predictive Control in C++ to drive a vehicle around the track in a [simulator](https://github.com/udacity/self-driving-car-sim). Escentially, the MPC manipulates the steering wheel and throttle to minimize the cross track error (CTE) and orientation error, while maintaining constant vehicle speed, in presense of 100 ms actuation latency.

[//]: # (Image References)

[image1]: ./images/x.png
[image2]: ./images/y.png
[image3]: ./images/psi.png
[image4]: ./images/v.png
[image5]: ./images/cte.png
[image6]: ./images/epsi.png
[image7]: ./images/input.png

---

## Vehicle Model

A sixth order kinematic model is considered in this project, which is given by:

![alt text][image1]

![alt text][image2]

![alt text][image3]

![alt text][image4]

![alt text][image5]

![alt text][image6]

where the states are x and y positions, oridentation, velocity, cross track error, and orientation error. The inputs are steering angle and acceleration, subject to te constraints:

![alt text][image7]

All the state and input variables are in SI units.

## MPC Parameters

The vehicle reference speed is set to be 17.88 m/s (40 mph). The prediction horizon `T` is chosen to be 3 seconds, which corresponds to 53.64 m look ahead distance. Beyond that horizon, the environment cannot be reliably predicted.


## Waypoints Fitting and Preprocessing


## Latency
