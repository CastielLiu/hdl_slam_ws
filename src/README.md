# run
***建图***
rosbag play --clock -r 0.5 2020-07-07-23-03-31.bag  ll2utm_odom:=/gps_odom  回放rosbag 
roslaunch hdl_graph_slam slam.launch  建图
rosservice call /hdl_graph_slam/save_map "utm: false resolution: 0.1  destination: '/home/map.pcd'" 保存地图


