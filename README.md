# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
* Brian Erickson

---
## Overview
Model Predictive Control (MPC) uses a model of how a vehicle will travel through a state space given actuations, a cost function to measures how well a set of planned actuations will meet a goal, and a solver to find an optimal set of actuations to minimize the cost function.

The model I used was based on fixed steering angles between time steps, fixed acceleration between time steps, and a bicycle style steering.  At each time step, the pose is updated based on the steering angle and any given acceleration.  The actuations are the steering angle and acceleration.  The state variables are the x and y position of the car, the current yaw angle of the car, and the current velocity of the car.

Here is how the state udpate is done:

           // get actuations (unflatten)
          auto & a = actuations[actuators_a + i * n_actuators];     // acceleration
          auto & delta = actuations[actuators_delta + i * n_actuators]; // wheel angle

          // update state at time step
          x += v * CppAD::cos(psi) * dt;
          y += v * CppAD::sin(psi) * dt;
          psi += v/Lf * delta * dt;
          v += a * dt;


I chose to use a timestep of 100ms and 10 time steps.  This means that the time horizon is about 1 second.  I chose the 100ms because it matches the latency that we are expected to achieve.  I chose the low number of time steps because it keeps the execution fast and higher time horizons didn't help to stabilize the car.  I tried finer time steps, such as 50ms, and longer time horizons up to several seconds, but they didn't improve performance.  In addition, sometimes the longer time horizons would cause bad behaviors like the car looping back on the track instead of going forward. Part of this could be because the waypoints presented from the simulator have a limited range in front and behind the car.

I used a polynomail to fit the waypoints.  This provided a smooth curve to use in calculations including calculations for cross track error and heading errors.

To make the car drive well with a 100ms lag, I made two adjustments.  The first was to use the second set of actions, that is, actuations 100ms from the initial reading.  The second thing I did was to ensure smooth steering in my cost function.  I did this by punishing high variability in steering angles between time steps.  It turns out that this had a much bigger effect on stability with high lag / high speed, than simply skipping 100ms of actuations.  Without these modifications for lag, the car would become unstable and drive off of the track at higher speeds with higher lags.



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* sudo apt-get install libcoin80v5 (stops runtime error complaining about libcoinmumps.so.1)
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.tgz <s>the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.</s>
    * You will need fortran installed `sudo apt-get install gfortran`
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh /home/<user>/Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

