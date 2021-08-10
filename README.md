# robot_script
## Setup Instructions
### Enviroment setup
See my google docs for setting up ROS2 and an RT_PREEMPT patched machine
### Set up workspace and clone repos
1. `mkdir` and `cd` into a directory to contain the workspace folder. In my case `~/Documents/robot_script_ws`
2. Clone the [Machines in Motion Lab treep package](https://github.com/machines-in-motion/treep_machines_in_motion). In my case
```
git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
```
3. Use treep to clone the [real_time_tools](https://github.com/machines-in-motion/real_time_tools) repo and all of its dependencies:
```
treep --clone REAL_TIME_TOOLS
```
Do the same for the [odri_control_interface](https://github.com/open-dynamic-robot-initiative/odri_control_interface) repo:
```
treep --clone ODRI_CONTROL_INTERFACE
```
Afterwards, my directory looks like
```
robot_script_ws/
	treep_machines_in_motion/
	workspace/
		src/
			googletest/
			master-board/
			mpi_cmake_modules/
			odri_control_interface/
			pybind11/
			real_time_tools/
			yaml_utils
```
4. Clone this repo
```
cd workspace/src/
git clone https://github.com/yunifuchioka/robot_script.git
```
then my directory becomes
```
robot_script_ws/
	treep_machines_in_motion/
	workspace/
		src/
			googletest/
			master-board/
			mpi_cmake_modules/
			odri_control_interface/
			pybind11/
			real_time_tools/
			robot_script/
			yaml_utils
```

### Run script
1. Switch to root, which is necessary for the network communcation
```
sudo -s
```
2. Source ROS2 if this isn't done automatically in the `.bashrc`. In my case
```
source /opt/ros/dashing/setup.bash
source /opt/openrobots/setup.bash
```
3. `cd` to the workspace directory (`cd ~/Documents/robot_script_ws/workspace` in my case), then build with colcon
```
colcon build
```
4. Source the setup file
```
source install/setup.bash
```
5. Run the script
```
ros2 run robot_script main MY_INTERFACE
```
where `MY_INTERFACE` if the name of the interface, obtained from running `ifconfig`.