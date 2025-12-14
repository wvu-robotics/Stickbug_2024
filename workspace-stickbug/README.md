# Stickbug Project

[![MIT License][license-shield]][license-url]


<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
     <ul>
        <li><a href="#simulation">Simulation Environment</a></li>
        <li><a href="#real_robot">Real Robot</a></li>
      </ul>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>


<!--About The Project -->
## About the project

Stickbug is a robot consisting of 7 robots, 1 drivebase and 6 robot arms. Each robot arm runs on its own raspberry pi, and the drivebase is the main computer that everything is launched from. 

currently two missions are inplace, teleop and drive to waypoint and manipulator to goal. 

<p align="right">(<a href="#top">back to top</a>)</p>

## Getting Started

The code was tested in a  Ubuntu 20.04 machine with ROS Noetic

### Prerequisites
  Suggested to install everything inside a python 3.9 virtual environment (such as coda env).
  
  Display Link for running the usb screen:
  ```sh
  sudo chmod +x displaylink-driver-5.5.0-59.151.run   #(from https://www.synaptics.com/products/displaylink-graphics/downloads/ubuntu)
  ```
  
  Graphic Card dependencies:
  ```sh  
   apt-get install libgl1-mesa-glx libegl1-mesa libxrandr2 libxrandr2 libxss1 libxcursor1 libxcomposite1 libasound2 libxi6 libxtst6
   sudo apt-get install libgl1-mesa-glx libegl1-mesa libxrandr2 libxrandr2 libxss1 libxcursor1 libxcomposite1 libasound2 libxi6 libxtst6
  ```
  Install nvidia drivers and CUDA:
  
  Install ZED2 SDK and its ros-wrapper and examples:
  ```sh
  python -m pip install cython numpy opencv-python pyopengl
  sudo chmod +x ZED_SDK_Ubuntu20_cuda11.5_v3.7.1.run 
  git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
  git clone https://github.com/stereolabs/zed-ros-examples.git
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make
  ```
  
  Install Realsense2 (librealsense + ros package)
  https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
  
  Install ros-noetic driver packages
  ``` sh
  sudo apt install ros-noetic-driver-base 
  sudo apt install ros-noetic-driver-common 
  sudo apt install ros-noetic-effort-controllers
  sudo apt install ros-noetic-move-base

  ```
  



### Installation
   ```sh
   git clone git@github.com:wvu-robotics/workspace-stickbug.git
   cd workspace-stickbug
   wstool init src stickbug-ssh.rosinstall 
   rosdep install --from-paths src --ignore-src -r -y
   ###Install  
 
   catkin_make -k
   echo "done installing"
   
   ```
### Update
   in the workspace-stickbug folder
   ```sh
  wstool update -t src
  catkin_make -k
  echo "done updating"
   
   ```

<p align="right">(<a href="#top">back to top</a>)</p>

## Usage
This repository contains a set of instructions to run the Stickbug project on the real robot  

to launch teleop control use 
```sh
  roslaunch stickbug_missions stickbug_teleop.launch
```
to launch waypoint control and manipulator go to pose use 
```sh
  roslaunch stickbug_missions simple_autonomy.launch
```
NOTE: to switch to simulation change the argument "use_hardware" to false

## Roadmap
#### Simulation

#### Real Robot
<p align="right">(<a href="#top">back to top</a>)</p>


## Contributing
Contribution to the project are greatly appreciated. 

when contibuting:  
 IF: code can be reused by another robot/project, place in its own package, and include into the .rosinstall here  
 ELSE IF: it is only usable by the drivebase, include into drivebase_missions package  
 ELSE IF: it is only usable by a manipulator, include into manipulator_missions package  
 ELSE: it involves communication or usage between agents include into the stickbug_missions package  
 
 REMEMBER!!! ensure that each agent (arm or drivebase) should still remain single agents that could be run seperately

## License

Distributed under the BSD3 License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#top">back to top</a>)</p>

## Contact
Chris Tatsch ca0055@mix.wvu.edu
Trevor Smith trs0024@mix.wvu.edu 

<p align="right">(<a href="#top">back to top</a>)</p>

## Acknowledgements
This project was funded by the *** TODO (Add funding agency)

<p align="right">(<a href="#top">back to top</a>)</p>

[license-url]: https://github.com/wvu-robotics/LICENSE.txt
[license-shield]: https://img.shields.io/github/license/wvu-robotics/stickbug






