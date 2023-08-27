# ME495 Sensing, Navigation and Machine Learning For Robotics
* Liz Metzger
* Winter 2022

# Package List
This repository consists of several ROS packages
- nusim - a package that runs a node serving as a simulator and visulizer of turtlebots in RVIZ
    nodes:
        * nusim.cpp
    launchfiles:
        * nusim.launch.xml

- nuturtle_description - package for displaying one or multiple turtlebots in Rviz
    nodes:
        * circle.cpp
        * odometry.cpp
        * turtle_control.cpp
    launchfiles:
        * start_robot.launch.xml

- turtlelib - C++ library for performing 2D rigid body transformations and other functionalities
    nodes:
        none
    launchfiles:
        * load_all.launch.xml
        * load_one.launch.py

# Turtlelib
This library has functions to compute the kinematics of the turtlebot.
- Header files:
    * diff_drive.hpp
    * rigid2d.hpp
- Libraries:
    * diff_drive.cpp
    * frame_main.cpp

# Live Testing: Part F
Here is a video of my robot running with my circle and teleop node:



https://user-images.githubusercontent.com/113066141/217866764-2a2201a1-5be6-4977-8eb0-35b0351dba6b.mp4




My ododmetry started as follows:
    position:
        x: 0.065730
        y: 0.065530
    orientation:
        w: 0.7081819

it ended as follows:
    position:
        x: -0.017951
        y: -0.016530
    orientation:
        w: 0.6703828

So the difference between them is:
    position:
        x: 0.083681
        y: 0.08206
    orientation:
        w: 0.0377991



