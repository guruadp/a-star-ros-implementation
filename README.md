# A* ROS implementation

## Description

## Authors
1. Guru Nandhan Appiya Dilipkumar Peethambari - guruadp(119183281)<br />
2. Vignesh Ravichandran Radhakrishnan - rr94(119144060)

## Installation
Install Python3

https://www.python.org/downloads/

Install Required Libraries
```
pip install numpy pygame sortedcollections
```
Install ROS noetic by following the commands in the website

http://wiki.ros.org/noetic/Installation/Ubuntu

### Cloning Repo

Create a workspace folder called catkin_ws (it can be any name)

```
mkdir catkin_ws && cd catkin_ws
```

Clone the repo in the workspace and rename the folder as src
```
git clone https://github.com/guruadp/a-star-ros-implementation.git
```

Building the package

Make sure your are in the workspace folder

```
caktin_make
```

Source the folder
```
source devel/setup.bash
```

To launch

```
roslaunch a_star a_star.launch
```