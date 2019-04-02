# dvrk_carla

## CARLA Simulatior intergation for the da Vinci Research Kit

This ROS-based package makes possible to use the [CARLA Simulator](http://carla.org/) on the [DVRK](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki).

## Usage of the Arduino programs

The Arduino programs use the [rosserial_arduino](http://wiki.ros.org/rosserial_arduino) interface to ROS.

After the program is loaded onto the board, the following node has to be launched to publish data to the ROS environment:

    rosrun rosserial_python serial_node.py /dev/ttyUSB0

