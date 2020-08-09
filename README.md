
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

Then,
```
cd ~/your_ws/src/fly_bot/src
chmod u+x control.py
chmod u+x PID.py
```

6.Execute the following command to build into your ROS workspace
```
cd ~/your_ws
catkin_make
```

Once installed, close the terminal. Open another terminal and load the Hexacopter into gazebo simulator
```
roslaunch fly_bot Kwad_gazebo.launch
```

You should see the Hexacopter fly upwards while stabilizing itself.

```
rosrun fly_bot control.py
```

Inspiration was taken from this [Quadcopter Simulation](https://github.com/NishanthARao/ROS-Quadcopter-Simulation). Thank You!


