# AutoModelCar CIC IPN
Welcome to the CIC's AutoModel Car gitHub. This repository contents the code for the next modules:

- Image processing and camera adjustment (C++ and Python).
- Lane follower (C++ and Pyhton).
- Intersection detector (C++ and Pyhton).
- Obstacke detection (C++ and Pyhton).
- Joystick (C++).

The ROS distro used is Indigo along with Ubuntu 14.10 LTS. All the C++ and Pyhton Modules are included in different ROS packages. The launch files are also included.

**IMPORTANT: Before starting, make sure you have ROS and all it's deppendencies properly installed on your PC! Otherwise, visit the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).**

## Cloning the repository
In order to start working with the code, first clone the repository by typing:

> git clone https://github.com/Conilo/automodelcar-cic-ipn.git

## Establish communication with the car
TODO

## Build the code:
Then, it's necesarry to compile the code (also after modifiying any file or node source). To do so, type:

> bash compile.bash

To compile the code in "release mode" add the `-s` flag:

> bash compile.bash -s

## Run the code

There are different run modes available, depending on the function needed. The next subsections explain each mode functionalities and how to run them.

### Camera adjustment mode

To run the camera adjustment mode, type:

> bash start.bash -cm

A debug window will be displayed with a chessboard layout (see figure 1). To adjust the camera, you will need a printed chesboard pattern of 35x35 [cm]. Lay down the printed pattern in front of the camera and modify the parameters in the camera calibration launch file in order to match the chessboard pattern with the one displayed. Those parameters are:

- Pixel to cm ratio in the X-axis.
- Pixel to cm ratio in the Y-axis.
- Scaling factor for the X-axis.
- Scaling factor for the Y-axis.
- Four points to wrap the image in birdview.

![](img/calibration_window.png)
Figure 1: Chessboard pattern displayed on camera adjustmen mode.

### Autonomous mode
This mode launches all the nodes needed to run the car on aoutonomous mode for the next tasks:

TODO

## Run the code with bags
To run the code with bags on the PC, having a ROS master running is needed,  you can do it by typing:

> roscore

or 

> bash start.bash -mode

where "mode" is to be replaced with the desired mode. Then, to play the desired bag, type:

> bash play_bag.bash desired_bag

where the "desired_bag" is the file name without extention.

**IMPORTANT: before playing a ROS bag file, make sure that a bags/ folder with your bag inside it exists in your working space. Otherwise, you may need to modify the play_bag.bash file to adjust the path to your bags container folder.**

## Contact:
If you need more info about the code, please contact:

* Project Coordinator:
Erik Zamora Gómez (e-mail: ezamora1981@gmail.com).

* Project Manager: 
Cesar Gerardo Bravo Conejo  (e-mail: conilo@gmail.com).

Student assistants:
- Brenda Camacho Garcia (e-mail: brendacg616@gmail.com).
- Esteban Iván Rojas Hernández (e-mail: rojasesteban23@gmail.com).
