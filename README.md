# Extended Kalman Filter

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


[//]: # (Image References)
[sim]: ./writeup_images/sim.gif "sim"

![alt text][sim]

## Writeup

The goals / steps of this project are the following:
* Build a Kalman filter for Lidar and Extended Kalman Filter for Radar from the C++ boilerplate
* Test that the filter works with the simulator and the estimation values are lower than the RMSE outlined
* Summarize the results with a written report


#### Simulator

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

### Source code files

Followings are the files that are used to compute the filters.

* src/main.cpp
* src/FusionEKF.cpp
* src/kalman_filter.cpp
* src/kalman_filter.h
* src/tools.cpp

### Build & Run

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
  * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

#### Test Mode

To speed up the testing I've included extra -f (file) argument that can be optionally passed to the program. 
Following demonstrates the usage to feed the program with the dataset from file and output the result to a output.txt.

```sh
./ExtendedKF -f data/obj_pose-laser-radar-synthetic-input.txt > output.txt
```

The output file contains the csv data with tab separators. For each lines the first 4 values are the estimate of the px,py,vx and vy.
The subsequent of each 4 values are the ground truth, RMSE and boolean of whether the RMSE within the acceptable range.


#### Websocket Mode

By default the program use websocket protocol to communicate with the simulator.

**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---

## Other Important Dependencies

* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
