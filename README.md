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
then my directory becomes
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
1. Source ROS2 if this isn't done automatically in the `.bashrc`. In my case
```
source /opt/ros/dashing/setup.bash
```
3. `cd` to the workspace directory (`cd ~/Documents/robot_script_ws/workspace` in my case), then build with colcon
```
colcon build
```
4. Source the setup file
```
source install/setup.bash
```
5. Change the permission on `/dev/cpu_dma_latency`. This was taken from the [Machines in Motion Lab's rt_preempt setup script](https://github.com/machines-in-motion/ubuntu_installation_scripts/blob/master/rt-preempt/ubuntu18.04/install_rt_preempt_kernel.sh)
```
sudo chmod 0777 /dev/cpu_dma_latency
``` 
6. Run the script
```
ros2 run robot_script main
```
