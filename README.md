# Flightmare Simulator
## Description
This is an interface that connects ARPL Quadrotor Simulator with the Flightmare Interface

## Prerequisites
* Install ARPL_Quadrotor_control https://github.com/arplaboratory/arpl_quadrotor_control (ASK PROF. LOIANNO FOR PERMISSION FIRST BEFORE ACCESSING!!!)
* Install libraries
  ```
  sudo dpkg -i /path/to/flightmare_simulator/Ubuntu_2204_libs/libzmqpp-dev_4.1.2-0ubuntu2_amd64.deb
  sudo dpkg -i /path/to/flightmare_simulator/Ubuntu_2204_libs/libzmqpp4_4.1.2-0ubuntu2_amd64.deb
  ```
* Install Flightlib_ros: https://flightmare.readthedocs.io/en/latest/getting_started/quick_start.html#install-with-ros
* Download the ARPL Specific Unity build Repo (Note: we have our special version Unity game builds are too big for Github):https://drive.google.com/drive/folders/112k4-5et8ZBVgw8d-tvBS8-Xlhh1nVJN?usp=sharing
* Terminator - 

```
sudo add-apt-repository ppa:gnome-terminator

sudo apt-get update

sudo apt-get install terminator
```

## Installation
1. After install the pre-requisites and terminator.
2. Open Terminator 
3. catkin build flightmare_simulator


## Running Ubuntu 20.04
1. open the x86_64 file from the drive. NOT THE ONE FROM UZH.
2. Make sure NVIDIA X Server is on (Performance Mode) if using GPU. Otherwise the Unity will NOT offload to GPU  !!!!
3. run  roslaunch flightmare_simulator flightmare_sim.launch 
Controls just like ARPL_Quadrotor_control

## Running POP OS
1. open a terminal 
```
vim ~/.bashrc
export __NV_PRIME_RENDER_OFFLOAD=1;
export __GLX_VENDOR_LIBRARY_NAME=nvidia;
```
2. Save bashrc
```
source ~/.bashrc
/path/to/flightmare.x86_64
```



## Input/Output Topic
### Input Topic
* The quadrotor is updated based on the /quadrotor/odom
### Output Topic
* The Front Camera Images are published to /quadrotor/quadrotor_simulator_so3/unity_drone_cam/compressed
* Camera Matrix Info is published to /quadrotor/quadrotor_simulator_so3/unity_drone_cam/camera_info 
* Simulation is done in RVIZ and graphics are handled in Unity. Please realize that Unity ONLY HANDLES Graphics. 
* Point Cloud

## Files of Note
This is just a modified version of the simulator interface. quadrotor_simulator_base.hpp has all the Unity Mirroring and object spawning

## Future Options
Ability to control Apriltag spawn in .yaml file. 

## Known Bugs and Fixes (USE THE ARPL/flightlib repository ask for permission here)
1. Flightmare random disconnnects. This is caused by the function in /flightmare/flightlib/src/bridges/unity_bridge.cpp as handleOutput()

Replace Blocking message
```
sub_.receive(msg)
```
with Non blocking message
``` 
if (!sub_.receive(msg, true)) {
  	return false;
   }
   ```
   
