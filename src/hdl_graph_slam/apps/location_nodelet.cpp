#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <hdl_graph_slam/keyframe.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/registrations.hpp>

#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/Transform.h>
#include <eigen_conversions/eigen_msg.h>

#include <pcl/visualization/pcl_visualizer.h>
#include<boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <yaml.h>

namespace fs = boost::filesystem;

namespace hdl_graph_slam {

class LocationNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::PointCloud2> MySyncPolicy;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocationNodelet() 
  {
  	is_first_frame = true;
  }
  virtual ~LocationNodelet() {}

  virtual void onInit() 
  {
    NODELET_DEBUG("initializing location_nodelet...");
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();
    
    utm_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, utm_topic, 2));
    points_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, points_topic, 1));
    sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*utm_sub,*points_sub));
    sync->registerCallback(boost::bind(&LocationNodelet::cloud_callback, this, _1, _2));
    
    odom_pub = nh.advertise<nav_msgs::Odometry>("/location/odom", 1);
    load_keyframes();
  }

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    auto& pnh = private_nh;
    utm_topic = private_nh.param<std::string>("utm_topic","/gps_odom");
    points_topic = pnh.param<std::string>("points_topic", "/filtered_points");
    odom_frame_id = pnh.param<std::string>("odom_frame_id", "odom");
    keyframes_dir = pnh.param<std::string>("keyframes_directory", "");

    // The minimum tranlational distance and rotation angle between keyframes.
    // If this value is zero, frames are always compared with the previous frame
    keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 0.25);
    keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 0.15);
    keyframe_delta_time = pnh.param<double>("keyframe_delta_time", 1.0);

    // Registration validation by thresholding
    transform_thresholding = pnh.param<bool>("transform_thresholding", false);
    max_acceptable_trans = pnh.param<double>("max_acceptable_trans", 1.0);
    max_acceptable_angle = pnh.param<double>("max_acceptable_angle", 1.0);

    // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
    std::string downsample_method = pnh.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = pnh.param<double>("downsample_resolution", 0.1);
    if(downsample_method == "VOXELGRID") 
    {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = voxelgrid;
    } 
    else if(downsample_method == "APPROX_VOXELGRID") 
    {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } 
    else 
    {
      if(downsample_method != "NONE") 
      {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" <<std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
      boost::shared_ptr<pcl::PassThrough<PointT>> passthrough(new pcl::PassThrough<PointT>());
      downsample_filter = passthrough;
    }

    registration = select_registration_method(pnh);
  }
  
  void load_keyframes()
  {
    int keyframe_cnt = 0;
    fs::directory_iterator begin(keyframes_dir);
    fs::directory_iterator end;
    for(auto it=begin; it!=end; ++it)
    {
    	if(fs::is_directory(*it))
    		++keyframe_cnt;
    }
    
    keyframes.reserve(keyframe_cnt);
    
    for(size_t i=0; i<keyframe_cnt; ++i)
    {
    	std::stringstream path;
    	path << boost::format("%s/%06d/") % keyframes_dir % i;
    	
    	pcl::PointCloud<PointT>::Ptr point_cloud(new pcl::PointCloud<PointT>);
    	
    	if(pcl::io::loadPCDFile<PointT> (path.str()+"cloud.pcd", *point_cloud) == -1)
		    PCL_ERROR ("Couldn't read file %s *.*\n", (path.str()+"cloud.pcd").c_str());
    	
    	//KeyFrame(const ros::Time& stamp, const Eigen::Isometry3d& odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud);
    	KeyFrame::Ptr keyframe(new KeyFrame(ros::Time::now(),Eigen::Isometry3d::Identity(),0.0, point_cloud));
    	
    	YAML::Node data = YAML::LoadFile(path.str()+"data.yaml");
    	
    	auto odom = data["odom"].as<std::vector<double>>();
    	
    	Eigen::Map<Eigen::Matrix<double,4,4,Eigen::ColMajor> > odomMatrix(odom.data());
    	
    	keyframe->odom = odomMatrix;
    	
    	keyframe->accum_distance = data["accum_distance"].as<double>();
    	
    	keyframes.push_back(keyframe);
    }
    
    YAML::Node data = YAML::LoadFile(keyframes_dir + "/map_in_world.yaml");
    
    auto map_in_world_vector = data["matrix"].as<std::vector<float>>();
    
    Eigen::Map<Eigen::Matrix<float,4,4,Eigen::ColMajor> > map_in_worldMatrix(map_in_world_vector.data());
    map_in_world = map_in_worldMatrix;
    
    NODELET_INFO_STREAM("load keyframes ok ...");
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback( const nav_msgs::OdometryConstPtr& utm_odom_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
  {
  	//NODELET_INFO_STREAM("cloud_callback...");
//  	double now = ros::Time::now().toSec();
  	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    auto filtered_points = downsample(cloud);
    
  	if(is_first_frame)
  	{
  		if(!seach_matching(filtered_points))
  		{
  			NODELET_INFO_STREAM("Unfamiliar environment !!!");
  			return;
  		}
  		is_first_frame = false;
  		std::cout << "search first matching ok!" << std::endl;
  	}
  	
  	Eigen::Matrix4f pose = locating(filtered_points); //local_pose
  	
  	publish_odometry(cloud_msg->header.stamp, cloud_msg->header.frame_id, pose);
  	//std::cout << ros::Time::now().toSec()-now << std::endl;
  }
  
  bool seach_matching(const pcl::PointCloud<PointT>::ConstPtr& filtered_points)
  {
  	registration->setInputSource(filtered_points);
  	pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>()); 
  	std::vector<float> fitscores(keyframes.size(),100);
	
  	for(size_t i=0; i<keyframes.size() && ros::ok(); ++i)
  	{
//  	boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer1;
//	  	mViewer1.reset(new pcl::visualization::PCLVisualizer("src Viewer"));
//		mViewer1->setBackgroundColor (0, 0, 0); 
//		pcl::visualization::PointCloudColorHandlerCustom<PointT> colored1(filtered_points, 0, 255, 0); 
//		mViewer1->addPointCloud<PointT> (filtered_points,colored1, "src cloud"); 
//		mViewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "src cloud");
//		mViewer1->addCoordinateSystem (1.0);
//		pcl::visualization::PointCloudColorHandlerCustom<PointT> colored2(keyframes[i]->cloud, 255, 0, 0); 
//		mViewer1->addPointCloud<PointT> (keyframes[i]->cloud,colored2, "dst cloud"); 
//		mViewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "dst cloud");
//		mViewer1->spinOnce(200);
		
  		//NODELET_INFO_STREAM("matching "<< i << " frame ...");
  		std::cout << "search first matching " << i << "/" << keyframes.size() <<  std::endl;
  		registration->setInputTarget(keyframes[i]->cloud);
  		registration->align(*aligned, Eigen::Matrix4f::Identity());
  		if(!registration->hasConverged())
  		{
		  NODELET_INFO_STREAM("scan matching has not converged, ignore this frame!");
		  continue;
		}
		//NODELET_INFO_STREAM("scan matching has converged, score: "<< registration->getFitnessScore());
		fitscores[i] = registration->getFitnessScore();
  	}
  	
  	size_t best_frame_index = 0;
  	float min_score = 100.0;
  	for(size_t i=0; i<fitscores.size(); ++i)
  	{
  		if(fitscores[i] < min_score)
  		{
  			min_score = fitscores[i];
  			best_frame_index = i;
  		}
  	}
  	
  	std::cout << "best_frame_index: " << best_frame_index << "   score: " << fitscores[best_frame_index] << std:: endl;
  	
  	if(min_score > 1.0)
  		return false;
  		
  	current_match_index = best_frame_index;
  	return true;
  	
  }
  
  Eigen::Matrix4f locating(const pcl::PointCloud<PointT>::ConstPtr& filtered_points)
  {
  	static Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  	double now = ros::Time::now().toSec();
  	registration->setInputSource(filtered_points);
  	pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>()); 

	registration->setInputTarget(keyframes[current_match_index]->cloud);
	registration->align(*aligned, guess);
	
	float score = registration->getFitnessScore();
	Eigen::Matrix4f transform = registration->getFinalTransformation();
	//guess = transform;
	//std::cout << "registration cost: " << ros::Time::now().toSec() - now << std::endl;
  	std::cout << "current_match_index: " << current_match_index << std::endl;
  	
  	auto pose = keyframes[current_match_index]->odom.matrix().cast<float>() * transform;
  	
  	float max_dis2 = transform(0,3)*transform(0,3) + transform(1,3)*transform(1,3) + transform(2,3)*transform(2,3);
  	for(size_t index=current_match_index+1; true; index++)
  	{
 		index = index % keyframes.size();
  		auto  nex_trans = keyframes[index]->odom.matrix().cast<float>().inverse()*pose;
  		float dis2 = nex_trans(0,3)*nex_trans(0,3) + nex_trans(1,3)*nex_trans(1,3) + nex_trans(2,3)*nex_trans(2,3);
  		if(dis2 > max_dis2)
  		{
  			current_match_index = index==0? keyframes.size()-1:index-1;
  			break;
  		}
  		max_dis2 = dis2;
  		//NODELET_INFO_STREAM("try to update match index...");
  	}
  	
  	return pose;
  }

  /**
   * @brief downsample a point cloud
   * @param cloud  input cloud
   * @return downsampled point cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }

 

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, odom_frame_id, base_frame_id);
    odom_broadcaster.sendTransform(odom_trans);
	
	Eigen::Matrix4f global_pose = map_in_world * pose ; //global_pose
	
    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id;

    odom.pose.pose.position.x = global_pose(0, 3);
    odom.pose.pose.position.y = global_pose(1, 3);
    odom.pose.pose.position.z = global_pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = base_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub.publish(odom);
  }


private:
  // ROS topics
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> points_sub;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> utm_sub;
  std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;

  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster keyframe_broadcaster;
  tf::TransformBroadcaster gps_odom_broadcaster;

  std::string utm_topic;
  std::string points_topic;
  std::string odom_frame_id;
  std::string keyframes_dir;
  ros::Publisher read_until_pub;

  // keyframe parameters
  double keyframe_delta_trans;  // minimum distance between keyframes
  double keyframe_delta_angle;  //
  double keyframe_delta_time;   //

  // registration validation by thresholding
  bool transform_thresholding;  //
  double max_acceptable_trans;  //
  double max_acceptable_angle;

  // odometry calculation
  boost::optional<Eigen::Matrix4f> world2odom; // the transform from world to odom
  boost::optional<Eigen::Matrix4f> gps2base;  // the transform from gps to base
  Eigen::Matrix4f prev_pose_gps;               // previous pose from gps
  Eigen::Matrix4f pose_gps;                    // the pose get from gps
  Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe
  Eigen::Matrix4f keyframe_pose;               // keyframe pose
  ros::Time keyframe_stamp;                    // keyframe time
  
  std::vector<KeyFrame::Ptr> keyframes;

  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;
  
  tf::TransformListener tf_listener;
  
  bool is_first_frame;
  size_t current_match_index;
  Eigen::Matrix4f first_trans;
  Eigen::Matrix4f map_in_world;
};

}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::LocationNodelet, nodelet::Nodelet)
