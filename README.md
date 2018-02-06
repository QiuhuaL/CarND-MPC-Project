# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

This project is to implement Model Predictive Control to drive the car around the track in a simulator. 

## The motion Model

The state of the motion model includes:

px: X-position of the vehicle in the forward direction
py: Y-position of the vehicle in the lateral direction
psi: Orientation of the vehicle
v: Velocity of the vehicle

The actuators of the vehicle are:
psi: Steering angle
a: Acceleration

The motion model equations are:

px(t+1) = px(t) + v(t) * cos(psi(t)) * dt
py(t+1) = py(t) + v(t) * sin(psi(t)) * dt
psi(t+1) = psi(t) + v(t) / Lf *  Psi * dt
v(t+1) = v(t) + a * dt;
Where dt is the timestep between predictions and Lf is the distance between the front and the center of gravity of the vehicle, which determines its turning radius.


## Timesteps and Frequency

The time step value for the controller was manully adjusted and the final parameters chosen are N = 20 steps and dt = 0.05 seconds. 

I found that if the horion is too short, the vehicle would go out of bounds on sharp turns becasue it is too slow to react and if the horizon is too far away, the vehicle will drive too conservatively due to its planning too far away down the road. 


## The Cost Function

In the cost function, I considered various terms with differnt factors for optimization, including cte, epsi, velocity, steering angle delta, acceleration a, and change of the the steering and acceleraton to enforce smooth driving. 

    const double cte_weight = 500;  //cte error
    const double epsi_weight = 500;  // orientation error
    const double v_weight   = 1;     //velocity
    const double delta_weight = 5;   // control angle
    const double a_weight = 5;       //acceleration 
    const double delta_smooth_weight = 50000;  //steering smoothing
    const double a_smooth_weight = 5;   //acceration smoothing 


## Polynomial Fitting and MPC Preprocessing

The waypoints are mapped into vehicle space and then fitted to a order 3 polynomial.  In the vehicle space, the initial x, y position and orientation of the vehicle are all zeros. 

## Dealing with Latency

A latency of dt = 0.1 seconds between a cycle of the MPC controller and the actual actuation was used to make it more like real driving. The latency term was handled by predicting states of dt seconds later instead of the current state. This affacts how fast the controller reacts.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

