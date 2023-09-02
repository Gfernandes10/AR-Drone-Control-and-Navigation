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
cd ..
rosdep install tum_ardrone
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
catkin_make
```

