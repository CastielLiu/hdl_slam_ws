# run
***建图***
1. 回放rosbag
rosbag play --clock -k -r 0.5 xx.bag  ll2utm_odom:=/gps_odom
2. 建图
roslaunch hdl_graph_slam slam.launch
3. 保存地图
rosservice call /hdl_graph_slam/save_map "utm: false resolution: 0.1  destination: '/home/map.pcd'"

***定位***
1. 定位节点
roslaunch hdl_localization hdl_localization.launch map:='map.pcd'
2. 回放rosbag
rosbag play --clock -r 0.5 xx.bag  ll2utm_odom:=/gps_odom


