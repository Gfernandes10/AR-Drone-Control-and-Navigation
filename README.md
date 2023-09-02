# AR-Drone-Control-and-Navigation

###Dependencies 
``` bash
rosdep update --include-eol-distros
sudo apt-get install freeglut3 freeglut3-dev
sudo apt-get install ros-indigo-keyboard
```

###Installation 
``` bash
cd catkin_ws/src
git clone https://github.com/Gfernandes10/tum_ardrone.git
git clone https://github.com/Gfernandes10/tum_simulator.git
git clone https://github.com/Gfernandes10/AR-Drone-Control-and-Navigation.git
cp -r ~/catkin_ws/src/AR-Drone-Control-and-Navigation/controllers/camera_info ~/.ros/
cd ..
rosdep install tum_ardrone
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
catkin_make
```
###Simulation
To initialize the simulation and the navigation system use the command bellow in a terminal
``` bash
roslaunch controllers simulator.launch 
```
You can press the Takeoff or Land button to command the AR Drone. You can also use the keyboard, joystick or autopilot to control the drone. Follow the tum_ardrone package for more information. 

To initialize the controller use the command bellow
``` bash
roslaunch controllers controller.launch  
```
When you do so a popup window like the picture bellow you appear. 

![image](https://github.com/Gfernandes10/AR-Drone-Control-and-Navigation/assets/90433424/301fd368-03e1-496d-94ec-dc4e1139eab3)

Through this window you can intialize the controller. You need to send a command to take-off before start the controller. You need to select the popup window and press "space" in your keyboard in order to initialize the controller.  

When you initialize the controller it will automatically set the desired position as: [x,y,z,heading] = [0,0,0.4,0].

##Sending commands to the controller through ros services.
Set reference:
rosservice call /setref "x: 0.0
y: 0.0
z: 0.0
heading: 0.0"

Set circle command:
rosservice call /circ "radius: 0.0
angular_velocity: 0.0
height: 0.0" 

set lemniscate trajectory:
rosservice call /inf

You can stop the commands at anytime by pressing space while the popup window is selected.

### Laboratory experiments
To test the controller in the real dorne you need to run 
``` bash
roslaunch controllers ardrone_driver.launch
roslaunch controllers tum_ardrone.launch
```

ESCREVER SOBRE AUTOINIT
