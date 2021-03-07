# Overview

## Project Introduction

> Implements a PID controller in C++ to maneuver the vehicle around the track. The simulator provides the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

## Results

![S](images/finalize_parameters_example_run.gif)
> Sped up example of final parameter run (no dangerous manuevers)

### PID Algorithm

A PID controller uses three parameters to adjust steering; **P** ("proportional"), **I** ("integral"), **D** ("differential"). Together, these parameters can be adjusted to better steer the vehicle based on the associated errors measured in real-time.

The **P** parameter causes the vehicle to steer in proportion to the vehicles's distance from the lane center (CTE). If the vehicle is too far in a direction, it will more drastically correct its steering. If the vehicle is only slightly too far in a direction, it will only slightly correct its steering.

The **I** parameter corrects the bias in the CTE (such as steering drift). Without this term, the controller can have a difficult time reaching its ideal position.

The **D** parameter counters the controller to overshoot the desired position due to the **P** parameter. This parameter allows the vehicle to more smoothly the desired position.

### Choosing PID Parameter Coefficients

Coefficients were determined with trial and error methods until the vehicle could reasonably move around the track safely.

The final values were P=`-0.10`, I=`-0.0001`, and D=`-1.50`(with a throttle of `0.3`).

* **P**: Many runs saw a too slight of an overcorrection. This meant the parameter needed to be increased in magnitude.
* **I**: This parameter didn't needto be very large since it appears there was little bias in the system.
* **D**: This parameter was a very important one to counteract the **P** parameter. In initial tests, the **D** parameter was accidentally set to `0` and the vechicle would frequently overshoot its desired position (center of the lane). After adjusting and re-adjusting the parameter, the vehicle does a better job in more smoothly correcting the steering.

## Set Up: Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

---------

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.