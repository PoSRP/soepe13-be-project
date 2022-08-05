[![Build Status](https://github.com/PoSRP/soepe13-be-project/actions/workflows/latex.yaml/badge.svg?branch=master)](https://github.com/PoSRP/soepe13-be-project/actions)
[![Build Status](https://github.com/PoSRP/soepe13-be-project/actions/workflows/ros2.yaml/badge.svg?branch=master)](https://github.com/PoSRP/soepe13-be-project/actions)  

# B.Eng. Project
University of Southern Denmark  
Søren Riisom Pedersen  
B.Eng. Robotics  
Søren Riisom Pedersen: soepe13@student.sdu.dk  

## About the project
This is a bachelor project for 7th semester B.Eng. Robotics at the [University of Southern Denmark](https://www.sdu.dk/en).  
The project is developed in collaboration with [Kobots A/S](https://kobots.com/en/).  

The goal of the project is to integrate the EtherCAT library [SOEM](https://openethercatsociety.github.io/) 
into ROS2 in order to evaluate EtherCAT motor drivers using the CiA-402 communication profile 
inside a ROS2 environment. 

It will consist of two phases:
  - Phase 1: Integrating the SOEM library into ROS2
  - Phase 2: Applying the integration

The first phase of the project will be applicable to all EtherCAT devices 
while the seconds phase requires the connected device to have a feedback mechanism in the 
manufacturer specific part of the device protocol. 
Due to economic and time constraints the project will be developed using a single EtherCAT device, 
an [AMC FM060-5-EM](https://www.a-m-c.com/product/fm060-5-em/).

A good integration should support the following:
  - Importing new devices using device description files
  - Configuration of fields from device description
  - Saving configurations for offline device 
  - Applying saved configurations on online devices
  - Changing applied configurations of online devices
  - Movement control through one or more synchronous profiles

The application of the developed integration should include the following:
  - Graphical comparison of expected versus achieved movements
  - Estimation of select mechanical parameters
  - An automatic calibration algorithm using configured fields

To ease user interaction with developed features an RQt graphical interface plugin will be developed during both phases. 

### Phase 1 - Integrating SOEM into ROS2
  - Importing EtherCAT devices into ROS2
  - Configuring imported devices
  - Applying configurations to a connected device
  - Jogging a singular device

### Phase 2 - Applying the integration
  - Applying complex movement profiles using a synchronous profile
  - Analyzing achieved movements in order to estimate mechanical parameters
  - Comparing estimated values to "classical approaches"
  - Developing an automatic calibration algorithm 
  - Testing the developed calibration algorithm 
