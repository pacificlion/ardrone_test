# ardrone

Run the following codd
```
mkdir -p ~/catkin_ws/src
cd catkin_ws/src
git clone https://github.com/pacificlion/ardrone_test.git
git clone https://github.com/pacificlion/vicon_bridge.git
git clone https://github.com/pacificlion/ardrone_autonomy.git
cd ~/catkin_ws

```

## Build catkin 
```
catkin build
```
```
## Running a simulation of AR.Drone 2.0 traversing along predefined trajectory (8 shape) on Gazebo
Run the following code lines in different terminals

```
roscore
```
```
roslaunch cvg_sim_gazebo ardrone_testworld.launch
```
```
rosrun ardrone_test ardrone_test_node
```
Now, press w to takeoff
```
rosrun ardrone_test ardrone_test_node
```
Press t to start trajectory tracking. (data of desired and actual positions can be saved and plotted)

## Making an AR.Drone 2.0 traverse along a predefined trajectory (8 shape) IRL with the help of Vicon Motion Capture system
Run the following code lines in different terminals
Make sure your computer is connected to same LAN as Vicon's rig and wifi is connected to ardrone

```
roscore
```
```
roslaunch vicon_bridge vicon.launch
```
```
rosrun ardrone_autonomy ardrone_driver
```
```
rosrun ardrone_test ardrone_test_node
```
Now, press w to takeoff
```
rosrun ardrone_test ardrone_test_node
```
Press v to start trajectory tracking. (data of desired and actual positions can be saved and plotted)

# Credits:

Initial code for git repository package was created from [kaustubhsridhar/AR.Drone-nonlinear-control](https://github.com/kaustubhsridhar/AR.Drone-nonlinear-control)
