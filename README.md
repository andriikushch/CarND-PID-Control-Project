# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Goal
The goal of this project is to write a PID controller for the car on the road and get some practical knowledge about P, I and D components impact and experience in tuning hyperparameters.

![pid.gif](pid.gif)

video [pid.mov](pid.mov)

## P, I and D

### 1. P
High values for P will make a car oscillating around the middle of the road, small values will reduce the car's willing to move to the middle of the lane.

| Big  | Smal  |
|---|---|
| ![pid-bp.gif](pid-bp.gif)  | ![pid-sp.gif](pid-sp.gif)  |
| [video](pid-bp.mov)  | [video](pid-sp.mov)  |

### 2. I
High values for I will "collect" error impact very quickly and will drive car out from the road. This component is useful for a biased system, which is not a case for the simulator.

![pid-bi.gif](pid-bi.gif)
video [pid-bi.mov](pid-bi.mov)

### 3. D

High values for D will cause "sharp" turns:

![pid-bd.gif](pid-bd.gif)

video [pid-bd.mov](pid-bd.mov)


While low values will not compensate for increasing of deviation enough.

## Solution:
In my solution I've chosen these parameters for steer PID:

```c++
  double init_Kp = 0.09;
  double init_Ki = 0;
  double init_Kd = 0.09;
``` 

I used PID for throttle with parameters `0.6, 0, 4.5` to decrease speed in turns or when the car is about to leave the road.


---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).




