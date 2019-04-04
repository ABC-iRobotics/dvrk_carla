# dvrk_carla

## CARLA Simulatior intergation for the da Vinci Research Kit

This ROS-based package makes possible to use the [CARLA Simulator](http://carla.org/) on the [DVRK](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki).

## Usage of the Arduino programs

The Arduino programs use the [rosserial_arduino](http://wiki.ros.org/rosserial_arduino) interface to ROS.

After the program is loaded onto the board, the following node has to be launched to publish data to the ROS environment:

    rosrun rosserial_python serial_node.py /dev/ttyUSB0

## Usage of CARLA control script

Follow [Getting started|https://carla.readthedocs.io/en/latest/getting_started/] page of CARLA documentation to the point of starting the simulator with:

    ./CarlaUE4.sh

Then export path to CARLA Python API package located in CARLA folder, e. g.:

    export PYTHONPATH=$PYTHONPATH:~/CARLA_0.9.3/PythonAPI/carla-0.9.3-py2.7-linux-x86_64.egg

And start the `manual_control_ros.py` script.
