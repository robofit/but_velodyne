but_velodyne_proc
===============================

This package contains tools for Velodyne 3D laser scanner (tested with Velodyne HDL-32E). There are three main nodes:

* Laserscan Node - simulates conventional 2D scanner by using data from 3D LiDAR.
* Groundmap Node - estimates "safe ground" (i.e. not too rough, without high steps and other obstacles) around the robot.
* Cloud Assembler Node - combines several consecutive point clouds into a more dense cloud.

More detailed documentation can be found on: http://wiki.ros.org/but_velodyne_proc
