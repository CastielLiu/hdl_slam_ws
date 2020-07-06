1. 回放rosbsg并重映射gps_odom
	rosbag play --clock -r 0.3 2020-07-04-15-08-01.bag /ll2utm_odom:=/gps_odom

2. 启动定位程序并传入全局地图(pcd文件)
	 roslaunch hdl_localization hdl_localization.launch map:=/home/zwei/wendao/rosbag/data/2020-07-04-15-08-01.pcd
	
	

