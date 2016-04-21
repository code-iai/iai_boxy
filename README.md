# iai_boxy
A collection of description, launch, configuration ,and installation files for the Boxy robot at the Institute for Artificial Intelligence.

## Installation

### Installation in catkin workspace
Assuming that you have an existing catkin workspace at ```~/catkin_ws```, this is how you install Boxy's drivers:
```
rosdep update
cd ~/catkin_ws/src
wstool merge https://raw.githubusercontent.com/code-iai/iai_boxy/master/rosinstall/boxy_real.rosinstall
wstool update
rosdep install --ignore-src --from-paths .
cd ~/catkin_ws
catkin_make
```
