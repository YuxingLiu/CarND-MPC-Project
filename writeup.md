# Model Predictive Control Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The goal of this project is to implement an Model Predictive Control in C++ to drive a vehicle around the track in a [simulator](https://github.com/udacity/self-driving-car-sim). Escentially, the MPC manipulates the steering wheel and throttle to minimize the cross track error (CTE) and orientation error, while maintaining constant vehicle speed, in presence of 100 ms actuation latency.

[//]: # (Image References)

[image1]: ./images/x.png
[image2]: ./images/y.png
[image3]: ./images/psi.png
[image4]: ./images/v.png
[image5]: ./images/cte.png
[image6]: ./images/epsi.png
[image7]: ./images/input.png
[image8]: ./images/cost_fun.png

---

## Vehicle Model

A sixth order kinematic model is considered in this project, which is given by:

![alt text][image1]

![alt text][image2]

![alt text][image3]

![alt text][image4]

![alt text][image5]

![alt text][image6]

where the states are x and y positions, orientation, velocity, cross track error, and orientation error. The inputs are steering angle and acceleration, subject to the constraints:

![alt text][image7]

All the state and input variables are in SI units.

## MPC Parameters

The vehicle reference speed is set to be 17.88 m/s (40 mph). The prediction horizon `T` is chosen to be 3 seconds, which corresponds to 53.64 m look ahead distance. Beyond that horizon, the environment cannot be reliably predicted.

Then, three sets of timestep length `N` and elapsed duration `dt` are evaluated:

| N   | dt    | 
|:---:|:-----:| 
| 30  | 0.1   | 
| 15  | 0.2   | 
| 10  | 0.3   | 

From the simulation results, I found there is not too much performance gain with reduced `dt`. Therefore, `dt = 0.3` and `N = 10` are selected, in order to save the computation time.

After tuning, the cost function has the form:

![alt text][image8]

## Waypoints Fitting and Preprocessing

The waypoints are first transformed from map's coordinate to car's coordinate, and then fitted as a cubic function:
```cpp
int n_wp = ptsx.size();
Eigen::VectorXd x_wp(n_wp);
Eigen::VectorXd y_wp(n_wp);

for(int i = 0; i < n_wp; i++) {
  x_wp[i] = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
  y_wp[i] = -(ptsx[i] - px) * sin(psi) + (ptsy[i] - py) * cos(psi);
}

// Third order polynomial fit
auto coeffs = polyfit(x_wp, y_wp, 3);
```

The positions and orientation are set to origin in car's coordinate, the velocity is converted from mph to m/s, and the errors are computed in car's coordinate:
```cpp
double x0 = 0;
double y0 = 0;
double psi0 = 0;

// mph to m/s
v = v * 0.44704;

double cte = y0 - polyeval(coeffs, x0);
double epsi = psi0 - atan(coeffs[1] + 2*coeffs[2] * x0 + 3*coeffs[3] * pow(x0, 2));

// Current state
Eigen::VectorXd state(6);
state << x0, y0, psi0, v, cte, epsi;
```

The steering and throttle values are transformed to steering angle and acceleration, before sending to MPC:
```cpp
// Current input
double steering = j[1]["steering_angle"];
double throttle = j[1]["throttle"];
double delta = - steering * deg2rad(25);
double a = throttle * 1.0;
Eigen::VectorXd input(2);
input << delta, a;
```

## Latency

To account for the 100 ms latency, MPC will recalculate the initial states using the vehicle model:
```cpp
  if(latency > 0) {
    double x0 = state[0];
    double y0 = state[1];
    double psi0 = state[2];
    double v0 = state[3];
    double cte0 = state[4];
    double epsi0 = state[5];

    state[0] = x0 + v0 * cos(psi0) * latency;
    state[1] = y0 + v0 * sin(psi0) * latency;
    state[2] = psi0 + v0 / Lf * input[0] * latency;
    state[3] = v0 + input[1] * latency;
    state[4] = cte0 + v0 * sin(epsi0) * latency;
    state[5] = epsi0 + v0 / Lf * input[0] * latency;
  }
```
