# CUBE-RT (Cube Relocation Technology)
## METR4202 2022 team17 repo
Public code repo for 2022 run of METR4202 with the project course. 

The code base has henceforth been titled CUBE-RT, standing for "Cube Relocation Technology". This project required the implementation a robotic arm to be used within an airport to deliver passengers luggage from a moving conveyor to specified drop zones. 

This was completed on a small scale using blocks and aruco tags (also known as fiducial markers).

## Team members

List of team members sorted by last name

| Team member | student ID |
| :------------: | :-----------: |
| Alex Anchivilca Baltazar  | 46151544   | 
| Jack Barton | 46466293 | 
| Caleb Frew | 45880959 | 
| Shyaam Raniga | 45807545 | 
| Niall Waller | 45919397 | 

## Hardware and Software Used: 

Python 3: Programming language used for the ease of programming and variety of packages available for it.

The Robot Operating Software (ROS) was a library in python used for interaction between code and physical systems. 

Raspberry Pi: Used as the main Linux computer.

VS code: Development environment.

## Install:

To get this code you can either use [git](https://git-scm.com/downloads) to clone the repo or [download](https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository). It is highly recommended to use git as to increase in debugging and rising of bugs or issues,

## Code breakdown

The code is broken down such that the code database is modular; allowing for new functionality to be added and old functionality to be removed with (_*hopefully*_) no issues. This breakdown of general code can be seen below 

1. CUBE-RT 
    1. Launch
        1. IO.launch
        2. TASK1.launch
        3. TASK2.launch
        4. TEST.launch
    2. scripts
        1. Gripper_SNODE.py
        2. bash_init
        3. constants.py
        4. helpers.py
        5. task1.py
    3. CMakeList.txt
    4. README.md
    5. Package.xml

## Utilisation / Launching the Robot
1. Run the terminal
2. Using the computer's terminal, 
3. Enter into the *catkin_ws* workspace and use *roslaunch* 
4. Enter 
5.  



