# Flightmare Simulator
## Description
This is an interface that connects ARPL Quadrotor Simulator with the Flightmare Interface

## Prerequisites
* Install ARPL_Quadrotor_control https://github.com/arplaboratory/arpl_quadrotor_control (ASK PROF. LOIANNO FOR PERMISSION FIRST BEFORE ACCESSING!!!)
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


## Running
1. open the x86_64 file from the drive. NOT THE ONE FROM UZH.
2. run  roslaunch flightmare_simulator flightmare_sim.launch 
Controls just like ARPL_Quadrotor_control

## Input/Output Topic
### Input Topic
* The quadrotor is updated based on the /quadrotor/odom
### Output Topic
* The Front Camera Images are published to /quadrotor/quadrotor_simulator_so3/unity_drone_cam/compressed
* Camera Matrix Info is published to /quadrotor/quadrotor_simulator_so3/unity_drone_cam/camera_info 
* Simulation is done in RVIZ and graphics are handled in Unity. Please realize that Unity ONLY HANDLES Graphics. 

## Files of Note
This is just a modified version of the simulator interface. quadrotor_simulator_base.hpp has all the Unity Mirroring and object spawning

## Future Options
Ability to control Apriltag spawn in .yaml file. 

#UBUNTU 22.04 PROBLEMS!!!!
The packages available aren't there. Download and manually install .dev files from these repositories
sudo apt-get install libzmq3-dev
https://pkgs.org/download/libzmqpp4

## Known Bugs and Fixes
1. Flightmare random disconnnects. This is caused by the function in /flightmare/flightlib/src/bridges/unity_bridge.cpp as handleOutput()

Replace Blocking message
```sub_.receive(msg)
with Non blocking message
``` if (!sub_.receive(msg, true)) {
  	return false;
   }
   
