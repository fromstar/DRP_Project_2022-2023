# DRP_Project_2022-2023

In this work a simulation on the detection of CO2 concentration in the air by agents distributed in an unknown environment is implemented.
Each agent is equipped with an UWB sensor for communication, a LiDar sensor for environment detection and a CO2 gas sensor for air quality detection. 
The agents explore the map using a common reference system created via MDS method in order to share the information in the same reference system and correct the read CO2 concentration using a Kalman filter.
At the end of the exploration the data are put together in order to build a global map with the air quality.

## Prerequisites
* MATLAB
* MATLAB Libraries:
  * Image Processing Toolbox V11.6
  * Statistics and Machine Learning Toolbox V12.4
  * Global Optimization Toolbox V4.7
  * Robotics System Toolbox V4.0
  * Navigation Toolbox V2.3
  * Control System Toolbox V10.12
  * Antenna Toolbox V5.3
  * Curve Fitting Toolbox V3.8
  * Deep Learning HDL Toolbox V1.4
  * Fixed-Point Designer V7.5
  * System Identification Toolbox V10.0
  * Medical Imaging Toolbox V1.0
  * Optimization Toolbox V9.4
  * Computer Vision Toolbox V10.3
* MATLAB CODER V5.5
* Simulink 

## Execute
Add to the path with matlab the folder "src/" and its subfolders and then just execute the main.m file in the src folder.
