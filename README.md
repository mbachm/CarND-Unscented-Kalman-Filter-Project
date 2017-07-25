# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

Overview
---
This is my implementation of project 2 of term 2 of the Udacity - Self-Driving Car NanoDegree . You can find the original repo under [CarND-Unscented-Kalman-Filter-Project](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project). 

How to run this project
---
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

NIS plots
---
After my unscented kalman filter was successfully executed, it will provide 2 datafiles for NIS calculation:

* NIS_lidar_data_file.txt
* NIS_radar_data_file.txt

With the `generate_NIS_graphics` python script you can generate your own plot of the NIS measurements for radar and lidar of my unscented kalman filter. The plot will be saved in the file `NIS_plot.png`.