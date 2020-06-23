# Boxy Install guide

## Repositories

UR Driver .rosinstall

```yaml
- git:
    local-name: Universal_Robots_ROS_Driver
    uri: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
    version: master
- git:
    local-name: fmauch_universal_robot
    uri: https://github.com/fmauch/universal_robot.git
    version: calibration_devel
```





## PRs to merge

iai_boxy_bringup

* merge clean-launches to master
* kinect launch was never tested

iai_maps: better map

* but other is default

iai_dlr_intergration: arm traj_server.py and launchfile

* no gitolite access and not allowed to fork



https://github.com/code-iai/iai_robots/pull/27

## Installation Guide on Leela

Install the two components (Universal Robot, Boxy) in separate workspaces, first UR, then Boxy. For controlling the arms you need access to the dlr_intergration repository from Alexis.

### Universal Robot

The head is a UR3 arm, last updated in February 2020. Find the full installation an manual at https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.

We pull a specific version of the two needed UR repos from the iai_boxy rosinstall files.

```bash
# source global ros
source /opt/ros/melodic/setup.bash

# create a catkin workspace
mkdir -p ur_ws/src && cd ur_ws/src

# clone the UR repositories with wstool
wstool init
wstool merge https://raw.githubusercontent.com/code-iai/iai_boxy/master/rosinstall/universal_robots.rosinstall 

# install dependencies
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace
catkin_make

# source workspace
source ~/us_ws/devel/setup.bash
```

### Boxy

Make sure the UR workspace is built properly and the workspace overlaying is right. Check it with

```bash
echo $CMAKE_PREFIX_PATH
```

The variable should include the paths to ROS andUR. If not, source the workspace again.

```
# overlay above ur_ws
source ~/ur_ws/devel/setup.bash

# create catkin package
mkdir -p ~/boxy_ws/src && cd ~/boxy_ws
catkin init

# pull boxy repositories from rosinstall
cd ~/boxy_ws/src
wstool init
wstool merge https://raw.githubusercontent.com/artnie/iai_boxy/master/rosinstall/boxy_real.rosinstall
wstool update

# ask alexis for access to the iai_dlr_intergration repository and clone
git clone gitolite3@kif.ai.loc:iai_dlr_integration

# install dependencies via rosdep
rosdep update
rosdep install --ignore-src --from-paths .

# build the workspace
cd ..
catkin build

# source boxy_ws as top level workspace covering UR and ROS base
echo ~/boxy_ws/devel/setup.bash >> ~/.bashrc
source ~/.bashrc
```

For working on boxy we use environment files. Put this file in your home directory and name it rosenv.sh. Make it executable with `chmod +x ~/rosenv.sh`.

```
#!/bin/sh
source ${HOME}/rosws/devel/setup.bash
exec "$@"
```



## Bringup

Check the power switches on Leelas back. 1, 2, 3, 7, 12 are usually active. See the full explanation of all switches [here](https://toychest.ai.uni-bremen.de/wiki/ias:boxy_setup). Activate **at least** the power switches 1, 2, 3, 4, 5, and 7  to move the base and torso. 6 is for the UR head, 9, 10, 11 for the arms and 12 for the arm controller. All the launchfiles are located in `iai_boxy/iai_boxy_bringup`. The general launcher is `boxy_complete.launch`.  Notice the parameters around the top of the file.

```
  <arg name="with_base" default="true"/>
  <arg name="with_neck" default="false"/>
  <arg name="with_gripper" default="false"/>
  <arg name="with_kinect" default="false"/>
```

Depending on what components you need, you can switch on and off loading the corresponding launch files. Each switch launches a general file, which includes other launch files within the directories `base`, `gripper`, and `head`. The `kitchen.launch` is separate from the general launch files, since you only occasionally need the URDF of the environment. 

A roscore is usually running by on one of the users in a separate session in TurboVNC and only dies when Leela is shut down. If Leela seems to become slow, check `htop` for resource consumption and kill the user's roscore process when in doubt.

To move Boxy you need the E-Stop remote and the DS4 (Dual Shock) Controller.

Common work flow to start up Boxy:

* Switch on the relays you need (1, 2, 3, 4, 5, 7)
   * If you choose to use the UR neck (6) or the arms (9, 10, 11, 12), follow the instructions further below.
* Take the E-Stop remote, it should always be by your side.
* Connect to Leela via SSH and open byobu if you like
* **Home the torso:** Since the torso has no idea of its joint initial joint state, we need to drive it all the way up. Don't worry, it will stop by itself. But if you hear something weired, **hit the E-Stop hard**, it takes quite a force to push the red button. The screwing bar itself sometimes shrieks and vibrates, especially when the torso is low. 
   * Release the breaks: Pull the red button, wait a second until the LED fades, then push and hold the green button for a moment until your hear Boxy clicking. The corpus motors are now active. If it doesn't click, the E-Stop relay needs to be rebooted.
   * Check the cables in Leela and make sure nothing could pull anything when the torso drives up. Especially the cables from the power supply are prone to lay on top of the torsos black cable holder.
   * Execute `rosrun omni_ethercat home_torso.sh`, a low-level ethercat-based script to command specific ports to do something, in this case, screw the bar which the torso is mounted on. This is the only script able to move the torso without connecting the DS4 controller.
      * If it doesn't, check the error message, it probably says to restart ethercat with `sudo /etc/init.d/ethercat restart`. Do so.
   * The torso triggers a break on the top right side (from Boxys perspective) when it is high enough and stops immediately.
   * The `triangle_base_joint` is now at state 0.0. Moving the torso down decreases the joint state. 
* Hit the E-Stop. Always do, release only if you need to.
* DS4 connection: hit the PS4 Button on the middle of the DS4 controller to establish bluetooth connection. The back of the controller lights up blue if the connection is established. 
   * If not, make sure it's the right controller, then use `bluetoothctl` in your SSH session to scan, pair and trust the DS4 controller. 
   * If the light goes off, recharge. Please recharge the controller after using it, with a standard micro-USB cable, plugged into Leela.
* Launch Boxy: 
   * Make sure that
      * the E-Stop is hit.
      * the DS4's light is static blue
      * no cables are hanging in the way or lying on the floor
   * Launch `roslaunch iai_boxy_bringup boxy_complete.launch`. 
      * One warning says, the controller is not found. One is okay.
      * Recommending to restart ethercat should be followed. Kill the process and run `sudo /etc/init.d/ethercat restart`.
      * More warnings about missing scans might come up, because of the DS4 kill-switch is still active.
   * Release the E-Stop to activate the motors.
   * Hit the `square` button on the DS4 controller to release the **soft-run-stop**.
      * the warnings should now cease. 
* Moving Boxy:
   * There are two locks that prevent Boxy from moving, the hard E-Stop and the soft run-stop or kill switch of the DS4 controller. You can activate the kill-switch with `cross` and disable (free) it with `square`. Even the torso won't move when the soft stop is active



### Giskardpy (optional)

Whether you install Giskard on Leela or on your local machine (recommended), install the following python packages first:

```
sudo pip install pybullet
sudo pip install scipy==1.2.2 # this is the last version for python 2.7
sudo pip install casadi
sudo pip install sortedcontainers
sudo pip install hypothesis # only needed if you want to run tests
sudo pip install pandas
```

Now create the workspace and install Giskard. If you are interested in the latest development, choose the `devel` branch of giskardpy and giskard_msgs.

```bash
source ~/us_ws/devel/setup.bash                 # overlay on top of ur_ws, boxy_ws and ROS base
mkdir -p ~/giskardpy_ws/src                     # create directory for workspace
cd ~/giskardpy_ws                               # go to workspace directory
catkin init                                     # init workspace, you might have to pip install catkin-tools
cd src                                          # go to source directory of workspace
wstool init                                     # init rosinstall
wstool merge https://raw.githubusercontent.com/SemRoCo/giskardpy/master/rosinstall/catkin.rosinstall
                                                # update rosinstall file
wstool update                                   # pull source repositories
rosdep install --ignore-src --from-paths .      # install dependencies available through apt
cd ..                                           # go to workspace directory
catkin build                                    # build packages
echo ~/giskardpy_ws/devel/setup.bash >> ~/.bashrc  # source giskardpy_ws as overlaying on top
```

### 
