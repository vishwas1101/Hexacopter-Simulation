
# Hexacopter-Simulation
# Installation #


Open a terminal.
1. Initiate a workspace in your home directory or use your existing favorite one.
```
source /opt/ros/melodic/setup.bash 

mkdir -p ~/your_ws/src
cd ~/your_ws/src
catkin_init_workspace
```

2.Create necessary workspace files
```
cd ~/your_ws
catkin_make
```

3.Add this workspace to your linux environment by sourcing the setup file to .bashrc. Assuming you are inside the home directory, 
```
cd ~
gedit .bashrc
```
Add this line at the end of the file.
```
source ~/your_ws/devel/setup.bash
```

4.Create a ROS package in your workspace. We will call it fly_bot. Add the rospy and std_msgs dependencies
```
cd ~/your_ws/src
catkin_create_pkg fly_bot rospy std_msgs
```

Download all the folders and files into the folder fly_bot. i.e all the folders and files seen in this repo must be present inside the fly_bot. Donot create another folder inside the fly_bot with all theses files.

Note: You have to replace the existing src folder and CMakeLists and package files with this repo's folder and files. 
The folder hierarchy thus, must be:
```
your_ws/src/fly_bot
  -/config
  -/launch
  -/meshes
  -/src
  .
  .
  .
  -CMakeLists.txt
  -package.xml
  -urdf.rviz
```

Then,
```
cd ~/your_ws/src/fly_bot/src
chmod u+x controlnew.py
chmod u+x PID.py
```

6.Execute the following command to build into your ROS workspace
```
cd ~/your_ws
catkin_make
```

Once installed, close the terminal. Open another terminal and load the quadcopter into gazebo simulator
```
roslaunch fly_bot Kwad_gazebo.launch
```

```
rosrun fly_bot controlnew.py
```

You should see the Quadcopter fly upwards while stabilizing itself.

Alternatively, you can provide commands to individual motors (here, there is no PID control and stabilization of the Quad):
```
rostopic pub -1 /Kwad/joint_motor_controller/command std_msgs/Float64MultiArray "data: [50, -50, 50, -50, 50, -50]"
```
This provides a speed of:

i)   50 units to front_right motor

ii) -50 units to front_left motor

iii) 50 units to left motor 

iv) -50 units to back_left motor

v) 50 units to back_right motor

vi) -50 units to right_motor

Here, the negative sign denotes rotation in the opposite direction.


The pid values are in the /src/PID.py file in your fly_bot directory. Play around with the values to see some control theory in action!
