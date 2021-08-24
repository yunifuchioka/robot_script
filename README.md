# robot_script
## Setup Instructions
### Enviroment setup
See my google docs for setting up ROS2 and an RT_PREEMPT patched machine
### Install dependencies and set up project directory
1. `mkdir` and `cd` into a directory to contain all project files. In my case `~/Documents/robot_script_ws`
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
4. Download LibTorch from the [PyTorch website](https://pytorch.org/get-started/locally/). The options I chose were:

| Parameter | Value |
|:---:|:---:|
| PyTorch Build | Stable |
| Your OS | Linux |
| Package | LibTorch |
| Language | C++/ Java |
| Compute Platform | CPU |

Additionally, I chose the `cxx11 ABI`. Note that since this machine is running an `rt-preempt` kernel, Nvidia drivers are unsupported for it and the device should be CPU (and not CUDA).

5. Extract the zip file and move the resulting `libtorch` folder to the current directory. Then my directory becomes
```
robot_script_ws/
	libtorch/
		bin/
		build-hash
		build-version
		include/
		lib/
		share/
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
6. Create a `models` folder. This will be where all of the neural network models will be stored, both for pytorch and for torchscript.
```
mkdir models
```
Now my directory is
```
robot_script_ws/
	libtorch/
		bin/
		build-hash
		build-version
		include/
		lib/
		share/
	models/
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
7. Clone this repo into the colcon src directory
```
cd workspace/src/
git clone https://github.com/yunifuchioka/robot_script.git
```
finally, my directory becomes
```
robot_script_ws/
	libtorch/
		bin/
		build-hash
		build-version
		include/
		lib/
		share/
	models/
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

### Run main script
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

### Convert torch model to torchscript moel
In order for a pytorch neural network model to be readable from a C++ program, it must first be converted to a torchscript model. See the [Pytorch tutorial on loading a torchscript model in C++](https://pytorch.org/tutorials/advanced/cpp_export.html) for more details. Here are the steps to perform this conversion:
1. Copy the trained model file, say `my_trained_model.pt` for example, into the `models` folder. The directory should look like
```
robot_script_ws/
	libtorch/
	models/
		my_trained_model.pt
	treep_machines_in_motion/
	workspace/
```
2. From a non-sudo user, run
```
python src/robot_script/srcpy/convert_torchscript_model.py "my_trained_model"
```
Notes:
- I recommend doing this within a python virtual environment with pytorch installed
- The string argument is the name of the model file, without the `.pt` file extension

Afterwards, my directory looks like
```
robot_script_ws/
	libtorch/
	models/
		my_trained_model.pt
		my_trained_model_script.pt
	treep_machines_in_motion/
	workspace/
```
where `my_trained_model_script.pt` is the equivalent model, converted to a torchscript model so it can be loaded from C++

### Build and run the `test_torch` executable
1. Switch to root
```
sudo -s
```
2. Source ROS2 if this isn't done automatically in the `.bashrc`. In my case
```
source /opt/ros/dashing/setup.bash
source /opt/openrobots/setup.bash
```
3. Build the project. Due to the design of the libtorch cmake files, it's [necessary to pass the absolute path of libtorch as a cmake argument](https://github.com/pytorch/pytorch/issues/12449). To do this, run
```
colcon build --cmake-args -DCMAKE_PREFIX_PATH=$PWD/../libtorch
```
4. Source the setup file
```
source install/setup.bash
```
5. Run the script
```
ros2 run robot_script test_torch "my_trained_model"
```
The above script looks for a torchscript model at `models/my_trained_model_script.pt`. Note that it's not necessary to specify the `_script.pt` portion in the file name.