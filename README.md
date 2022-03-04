# ROS NODE: Learning to Segment Object Affordances on Synthetic Data for Task-oriented Robotic Handovers"

This repository contains the implementation of the ROS node for our paper "Learning to Segment Object Affordances on Synthetic Data for Task-oriented Robotic Handovers". With this repository task-oriented handover can be performed on the little helper robot with a robotiq gripper and an intel realsense D435.

The other affilated repositories for this paper can be found at.  

Main: https://github.com/HuchieWuchie/affnetDR  
Synthetic dataset generator: https://github.com/HuchieWuchie/affordanceSynthetic

## Installation instructions

### Pre-requisites:

0. OS: Ubuntu 18.04

1. Nvidia driver: 470.57.02
	 CUDA Version:  11.4

2. Install docker and nvidia-docker, guide here: https://medium.com/@linhlinhle997/how-to-install-docker-and-nvidia-docker-2-0-on-ubuntu-18-04-da3eac6ec494

3. Install ros-melodic (from here: http://wiki.ros.org/melodic/Installation/Ubuntu)

4. Additional dependencies:
```
sudo apt install gcc-6 g++-6
```

For a full explanation of the system in this repository please see the project report pdf "rob9report.pdf" in this repository.


### Installation

1. Pull docker image(s)
```
docker pull huchiewuchie/graspnet-rob9
```

2. Setup catkin workspace
```
mkdir -p ros_ws/src
cd ros_ws/src
git clone https://github.com/HuchieWuchie/AffNetDRROS.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git
git clone https://github.com/ros-industrial/robotiq.git
git clone https://github.com/daniellehot/ptu.git
git clone https://github.com/TAMS-Group/bio_ik.git
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -y
catkin_make -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.4 -DCMAKE_C_COMPILER=gcc-6 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```
For information on how to calibrate and establish connection to a UR5 robotic manipulator, please see the official ROS Universal Robots package that can be found here: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

3. Download pretrained weights

Download pretrained weights from: https://drive.google.com/file/d/1psCn_aT5KUyQDJrdxqR7GJgHeCewGokS/view?usp=sharing  

Place and rename the weights file to ros_ws/src/affordanceAnalyzer/scripts/affordance_synthetic/weights.pth

### Usage

In order to use this system, several ros nodes must be started. One complete launch file is on the feature list.

#### 1. Basics
```
roslaunch rob9 arm_bringup.launch
roslaunch rob9 utilities.launch
rosrun realsense_service server.py
rosrun affordance_analyzer server_synth.py
```

#### 2. Launch grasp_generator in docker, see docker/Readme.md for more information

##### Start docker container:
```
docker run --name graspnet-ros -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix --net=host --gpus all --rm huchiewuchie/graspnet-rob9 /bin/bash
```

##### Run ros node:
```
rosrun grasp_generator server.py
```

#### 3. Run experiment or main execution module:

```
rosrun moveit_scripts execute.py
```

##### 4. Command the robot

Once the execute.py script has analyzed the scene it will listen for commands on which tool and affordance to grasp on the topic "objects_affordances_id".

##### Trouble shooting

You might need to make the python scripts executeable with chmod +x

### Authors

Albert Daugbjerg Christensen  
Daniel Lehotsky  
Marius Willemoes Jørgensen  
Dimitris Chrysostomou
