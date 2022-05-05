# Flightmare Simulator
## Description
This is an interface that connects ARPL Quadrotor Simulator with the Flightmare Interface

## Prerequisites
* Install ARPL_QUadrotor_control https://github.com/arplaboratory/arpl_quadrotor_control
* Install Flightlib_ros: https://flightmare.readthedocs.io/en/latest/getting_started/quick_start.html#install-with-ros
* Download the ARPL Specific Ubuntu Repo: https://drive.google.com/drive/folders/1lf9O3XdgDuCrQljxSLuY83v6tVaWvWkQ?usp=sharing
* Terminator - 

```
sudo add-apt-repository ppa:gnome-terminator

sudo apt-get update

sudo apt-get install terminator
```

## Installation
* 1.After install the pre-requisites and terminator.
* 2.Open Terminator 
* 3.catkin build flightmare_simulator


## Running
1. open the x86_64 file from the drive. NOT THE ONE FROM UZH.
2. run  roslaunch flightmare_simulator flightmare_sim.launch 
Controls just like ARPL_Quadrotor_control

## INPUT/Output Topic
*The quadrotor is updated based on the /quadrotor/odom
*The Front Camera Images are published to /quadrotor/quadrotor_simulator_so3/unity_drone_cam/compressed
*Camera Matrix Info is published to /quadrotor/quadrotor_simulator_so3/unity_drone_cam/camera_info <- Development in progress

##Future Options
Ability to control Apriltag spawn in .yaml file. 
