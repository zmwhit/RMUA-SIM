# RMUA SIM
decision-making and plannning simulator for [RMUA](https://www.robomaster.com/zh-CN/robo/icra?djifrom=nav)
- Simulation of multi robot 
- Ideal kinematics model of nonholonomic and holonomic
- Freely customized structured obstacles

<img src="RMUA-SIM.gif" width="637" height="229" >

There is a hierarchical motion planner in [rmua_planning](rmua_planning) feature by:
- Motion mode managed using FSM
- Shorterst path searched using 2D Astar 
- Smooth path optimization using QP
- Speed optimization using QP to improve movement efficiency
- Glog for recording data
- Matplotlibcpp for online visualizing wave data
## Prerequisites
C++
```
osqp
OsqpEigen
g2o
glog
gflags
```
Python
```
numpy
pandas
pygame
matplotlib
```
## Build
```
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes -j16
```
## Run Simulation
```
roslaunch rmua_simulator sim.launch
```