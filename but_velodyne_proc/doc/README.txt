= ROS setup =
* Install required ROS packages, etc.
 sudo apt-get install ros-groovy-velodyne

= Hardware setup =
* Plug in the power adapter - the Velodyne sensor should start to rotate
* Plug in the ethernet cable and connect it with computer
* Set your ethernet IP address to something in 192.168.3.XXX range

 sudo ifconfig eth0 192.168.3.100

* Set the static route to the LIDAR sensor

 sudo route add 192.168.18.58 eth0

From now on, you should be able to debug recieving of LIDAR's UDP broadcast datagrams (using Wireshark for example).

= Publishing LIDAR's point cloud with ROS =
* After HW setup, you just need to run Velodyne launch file

 roslaunch but_velodyne_proc velodyne.launch

* Point cloud will be published on topic /velodyne_points
* In cloud_nodelet.launch file, you can modify min/max range of the sensor

