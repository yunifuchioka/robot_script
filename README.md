# robot_script
## Setup Instructions
### Enviroment setup
See my google docs for setting up ROS2 and an RT_PREEMPT patched machine
### Set up workspace and clone repos
1. `mkdir` a directory to contain the workspace folder. In my case `~/Documents/robot_script_ws`
2. Install [treep](https://gitlab.is.tue.mpg.de/amd-clmc/treep) and clone the [Machines in Motion Lab treep package](https://github.com/machines-in-motion/treep_machines_in_motion)
3. Use treep to clone the real_time_tools repo with all of its dependencies:
```
treep --clone REAL_TIME_TOOLS
```
Afterwards, my directory looks like
```
robot_script_ws/
	treep_machines_in_motion/
	workspace/
		src/
			googletest/
			mpi_cmake_modules/
			pybind11/
			real_time_tools/
```
4. Clone this repo
```
cd workspace/src/
git clone https://github.com/yunifuchioka/robot_script.git
```
the my directory becomes
```
robot_script_ws/
	treep_machines_in_motion/
	workspace/
		src/
			googletest/
			mpi_cmake_modules/
			pybind11/
			real_time_tools/
			robot_script/
```

### Run script
1. Source ROS2 if this isn't done automatically in the .bashrc
```
source /opt/ros/dashing/setup.bash
```
3. `cd` to the workspace directory (`cd ~/Documents/robot_script_ws/workspace` in my case), then build with colcon
```
colcon build
```
4. Source the setup file
```
source /install/setup.bash
```
5. Run the script
```
ros2 run robot_script main
```
