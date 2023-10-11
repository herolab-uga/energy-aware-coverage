# EAC
Energy-aware coverage

## Overview
EAC takes into account both robots's energy consumption rate and available energy level. It dynamically adjusts the allocation of regions, assigning smaller areas to robots with lower energy levels or faster depletion rates, and vice versa. EAC seeks to extend the overall lifespan of the multi-robot system while optimizing the achievement of mission objectives.


# Video Demonstration of the Experiments

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/NfVj7JEAVz0/0.jpg)](https://www.youtube.com/watch?v=NfVj7JEAVz0)


## Installation Requirement
Matlab 
*([Fast Bounded Power Diagram](https://www.mathworks.com/matlabcentral/fileexchange/56633-fast-bounded-power-diagram) MATLAB Add-on is also required, but it's built into this repository under third-party utilities (tp-utility) folder; you don't need to install)
* [Robotarium Matlab Installation]([https://pypi.org/project/robotarium-python-simulator/](https://github.com/robotarium/robotarium-matlab-simulator)https://github.com/robotarium/robotarium-matlab-simulator) 

## How to run
Place the repository within the "example" directory of Matlab Robotarium and execute the main script labeled as "EnergyAwareRobotarium.m."

## Parameters
You can set following paramters in [script](EnergyAwareRobotarium.m):
1. robots initial positions
2. density source locations (u1,u2)
3. threshold distance to centroid
4. velocity
5. density flag

## Core contributors

* **Aiman Munir** - PhD Candidate
* **Dr. Ramviyas Parasuraman** - Principal Investigator

## Heterogeneous Robotics (HeRoLab)

**Heterogeneous Robotics Lab (HeRoLab), Department of Computer Science, University of Georgia.** http://hero.uga.edu 

For further information, contact Aiman Munir aiman.munir@uga.edu or Prof. Ramviyas Parasuraman ramviyas@uga.edu

http://hero.uga.edu/
