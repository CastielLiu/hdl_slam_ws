#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <hdl_graph_slam/FloorCoeffs.h>

#include <hdl_graph_slam/SaveMap.h>
#include <hdl_graph_slam/DumpGraph.h>
#include <hdl_graph_slam/SaveOdom.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/ros_time_hash.hpp>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/keyframe_updater.hpp>
#include <hdl_graph_slam/loop_detector.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <hdl_graph_slam/map_cloud_generator.hpp>
#include <hdl_graph_slam/nmea_sentence_parser.hpp>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>


#include<message_filters/time_synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<std_msgs/String.h>



namespace debug {

class DebugNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,nav_msgs::Odometry,sensor_msgs::PointCloud2> MySyncPolicy;

  DebugNodelet() {}
  virtual ~DebugNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    // Get the node handle with the Multi Threaded callback queue. (provides this nodelets custom remappings and name)
    // 多线程句柄
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    // subscribers
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, "/odom", 256));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, "/velodyne_points", 32));
    utm_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, "/gps_odom", 256));
    
    sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*odom_sub,*utm_sub,*cloud_sub));
	sync->registerCallback(boost::bind(&DebugNodelet::cloud_callback, this, _1, _2,_3));
	
	debug_oub = nh.advertise<std_msgs::String>("debug",10);
  }

private:
  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg
   */
  void cloud_callback(const nav_msgs::OdometryConstPtr& odom_msg,
					  const nav_msgs::OdometryConstPtr& utm_msg,
					  const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) 
  {
  	std::string str1,str2,str3;
  	str1 = std::to_string(odom_msg->header.stamp.toSec());
  	str2 = std::to_string(utm_msg->header.stamp.toSec());
  	str3 = std::to_string(cloud_msg->header.stamp.toSec());
  	std_msgs::String msg;
  	msg.data = str1 + " " + str2 + " " + str3 + " ";
  	debug_oub.publish(msg);
  	
  	
  }

  
private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  ros::WallTimer optimization_timer;
  ros::WallTimer map_publish_timer;

  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> utm_sub;
  std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;
  
  ros::Publisher debug_oub;
  
};

}

PLUGINLIB_EXPORT_CLASS(debug::DebugNodelet, nodelet::Nodelet)
