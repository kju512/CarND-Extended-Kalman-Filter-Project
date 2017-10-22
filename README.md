# Extended Kalman Filter Project
This is a Self-Driving Car Engineer Nanodegree Program completed by Michael chen.This program implements an extended kalman filter.


This project involves the Udacity Self-Driving Car Engineer Nanodegree Program Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Running the program also need the library [uWebSocketIO],you can install it from [uWebSocketIO](https://github.com/uWebSockets/uWebSockets).


# Install and run
Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. Make a build directory: 'mkdir build'
2. enter into the directory: 'cd build'
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF `
5. Open the Term 2 Simulator,and select the first project 'Project 1/2:EKF/UKF',click button 'start'.
6. Under 'dataset1',you will get an output RMSE of [X:0.0973,Y:0.0855,VX:0.4513,VY:0.4399] at the last step(Timp Step:498).
   Under 'dataset2',you will get an output RMSE of [X:0.0726,Y:0.0967,VX:0.4579,VY:0.4966] at the last step(Timp Step:499).


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

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











