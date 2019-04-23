# dvrk_carla

## CARLA Simulatior intergation for the da Vinci Research Kit

This ROS-based package makes possible to use the [CARLA Simulator](http://carla.org/) on the [DVRK](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki).

## Setup envionment

The solution is tested on Ubuntu 16.04 LTS with ROS Kinetic [recommended by dVRK](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Development-Environment) from October 2017, so this platform is encouraged. The building process can be performed using catkin build tools for ROS (so use `catkin build`, but you should never `catkin_make`) by the following steps.

### Install ROS Kinetic

Type:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt update
    sudo apt install ros-kinetic-desktop-full

Initialize rosdep:

    sudo rosdep init
    rosdep update

Setup environment:

    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

#### Install dependencies for building packages

    sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools
    
### dVRK
 
Installthe [da Vinci Reserach Kit v1.5](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki), icluding the [cisst-saw](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros) and the [dvrk-ros](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild#dvrk-ros) packages, use do the following steps:

* install `cisst-saw` by folloing this [guide](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros)
* install `dvrk-ros` as seen in this [guide](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild#dvrk-ros)

### Build dvrk_carla

In your have installed dVRK, you should already have a catkin_ws directory set-up properly. Elsehow, do the following:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
    
If you alread have a catkin_ws, just download the sources:

    cd ~/catkin_ws/src
    git clone https://github.com/ABC-iRobotics/dvrk_carla
    
And build using `catkin build`:

    cd ~/catkin_ws
    catkin build dvrk_carla
    source devel_release/setup.bash    
    

## Usage of the Arduino programs

The Arduino programs use the [rosserial_arduino](http://wiki.ros.org/rosserial_arduino) interface to ROS.

After the program is loaded onto the board, the following node has to be launched to publish data to the ROS environment:

    rosrun rosserial_python serial_node.py /dev/ttyUSB0

## Usage of CARLA control script

You will need to download the latest release of CARLA from their [GitHub page](https://github.com/carla-simulator/carla/blob/master/Docs/download.md). Extract the archive to a folder of your choice. Home folder of current user is used in the examples below.

Next install Pygame and Numpy Python packages if you don not have them installed yet. On Ubuntu, it can be done with:

    sudo apt install python-pygame python-numpy

On other distributions you may consider installing it via pip only for current user:

    pip install --user pygame numpy

In a terminal window, switch the current directory to the path you extracted CARLA to and start the simulator with:

    ./CarlaUE4.sh

If you run into any problems during this step and to get more familiar with CARLA you may want to consult [Getting started](https://carla.readthedocs.io/en/latest/getting_started/) page of CARLA documentation this section was loosely based on.

To start control script which can be used with dVRK, export path to CARLA Python API package located in CARLA folder, e. g.:

    export PYTHONPATH=$PYTHONPATH:~/CARLA_0.9.3/PythonAPI/carla-0.9.3-py2.7-linux-x86_64.egg

And start the `manual_control_ros.py` script.

## Usage of the DVRK with steering wheel

Wheel segments, attachable to the MTMs can be printed using the STL file in the models folder. 

Start roscore:

    roscore

Launch the DVRK console with the two MTMs and foot pedals using the proper JSON file in a separate terminal, e.g.:

    rosrun dvrk_robot dvrk_console_json -j <path_to_JSON_files>/console-MTMR-MTML.json
    
Then launch the dvrk_steering_wheel.py script by:

    rosrun dvrk_carla dvrk_steering_wheel.py MTML MTMR
    
Move the handles to a comfortable position, and press COAG to fix wheel position. After this step the simulator is ready!
