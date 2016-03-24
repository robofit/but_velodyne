#! /bin/bash

for i in $(seq 0 4); do
	[ -f 00000$i.pcd ] || wget --no-check-certificate https://www.fit.vutbr.cz/~ivelas/files/kitti-sample/00000$i.pcd
done

roslaunch but_velodyne_odom collar_line_odometry.launch &

sleep 10

for f in 00000*.pcd; do 
	rosrun pcl_ros pcd_to_pointcloud $f /cloud_pcd:=/velodyne_points
	sleep 1
done
