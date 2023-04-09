# A* ROS implementation

## Description
This project deals with the implementation of A-star algorithm on a mobile robot for moving the Turtlebot3 (Burger) in the shortest path from source to goal point (user inputs) in a defined map of obstacles

## Authors
1. Guru Nandhan Appiya Dilipkumar Peethambari - guruadp(119183281)<br />
2. Vignesh Ravichandran Radhakrishnan - rr94(119144060)

## Installation
Install Python3

https://www.python.org/downloads/

Install Required Libraries and dependencies
```
pip install numpy pygame sortedcollections
```
Install ROS noetic by following the commands in the website

http://wiki.ros.org/noetic/Installation/Ubuntu

## Github Repo link
```
https://github.com/guruadp/a-star-ros-implementation.git
```

## Cloning Repo

Create a workspace folder called catkin_ws (it can be any name)

```
mkdir catkin_ws && cd catkin_ws && mkdir src && cd src
```

Clone the repo in the workspace(inside src folder)
```
git clone https://github.com/guruadp/a-star-ros-implementation.git
```

## Building the package

Make sure your are in the workspace folder

```
caktin_make
```

Source the folder
```
source devel/setup.bash
```

## Part1 - A-star (Pygame visualization)
For running part1 of phase2 i.e., Pygame Visualization, run the following command after entering the workspace
```
cd src/a-star-ros-implementation/a_star/scripts/
python3 a_star_phase_2_part_1.py
```
### Sample inputs for running part1

Enter clearance (in milli meters) : 100 </br>
Enter start x position (in meters) : 1.0 </br>
Enter start y position (in meters) : 0.5 </br>
Enter start orientation (in degrees): 0 </br>
Enter goal x position (in meters) : 5.5 </br>
Enter goal y position (in meters) : 1.2 </br>
Enter rpm1 : 50 </br>
Enter rpm2 : 100 </br>

## Part2 - ROS (Gazebo Visualization)
For running part2 of phase2 i.e., Gazebo Visualization, run the following command
```
roslaunch a_star a_star.launch 
```

### Sample inputs for running part2


## Output
### Video link for Phase2 Part1 : Pygame Visualization
```
https://drive.google.com/file/d/1VQNPc300kj1LL65xAfZy6Aw-AeKBYweF/view?usp=sharing
```
### Video link for Phase2 Part2 : Gazebo Visualization
