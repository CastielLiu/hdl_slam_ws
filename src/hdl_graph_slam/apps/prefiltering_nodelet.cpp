#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace hdl_graph_slam {

class PrefilteringNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;

  PrefilteringNodelet() {}
  virtual ~PrefilteringNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    points_sub = nh.subscribe(raw_points_topic, 64, &PrefilteringNodelet::cloud_callback, this);
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/prefiltering/filtered_points", 32);
  }

private:
  void initialize_params() 
  {
  	raw_points_topic = private_nh.param<std::string>("raw_points_topic","/velodyne_points");
    std::string downsample_method = private_nh.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);

    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = voxelgrid;
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" <<std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
    }
    use_outlier_filter = private_nh.param<bool>("use_outlier_filter", false);
    std::string outlier_removal_method = private_nh.param<std::string>("outlier_removal_method", "STATISTICAL");
    if(outlier_removal_method == "STATISTICAL") 
    {
      int mean_k = private_nh.param<int>("statistical_mean_k", 20);
      double stddev_mul_thresh = private_nh.param<double>("statistical_stddev", 1.0);
      std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_removal_filter = sor;
    } 
    else if(outlier_removal_method == "RADIUS") 
    {
      double radius = private_nh.param<double>("radius_radius", 0.8);
      int min_neighbors = private_nh.param<int>("radius_min_neighbors", 2);
      std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
      rad->setRadiusSearch(radius);
      rad->setMinNeighborsInRadius(min_neighbors);
      outlier_removal_filter = rad;
    } 
    else 
      std::cout << "outlier_removal: NONE" << std::endl;

    use_distance_filter = private_nh.param<bool>("use_distance_filter", true);
    distance_near_thresh = private_nh.param<double>("distance_near_thresh", 1.0);
    distance_far_thresh = private_nh.param<double>("distance_far_thresh", 100.0);
    height_thresh = private_nh.param<double>("height_thresh", 2.0);

    base_link_frame = private_nh.param<std::string>("base_link_frame", "base_link");
  }

  //判断tf变换是否为单位矩阵
  bool isTfTransformIdentity(const tf::StampedTransform& transform)
  {
    auto R = transform.getBasis();
    auto T = transform.getOrigin();

    if(R == R.getIdentity() && T.getX() ==0 && T.getY() ==0 && T.getZ() ==0)
      return true;
    return false;
  }
    
    
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  { 
	if(cloud_msg->data.size() == 0) 
		return;
    
    pcl::PointCloud<PointT>::Ptr source_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *source_cloud);
    
    pcl::PointCloud<PointT>::Ptr filtered = distance_filter(source_cloud);
    filtered = downsample(filtered);
    
    //std::cout << "pc size after filter:" << filtered->size() << std::endl;
    if(use_outlier_filter)
		filtered = outlier_removal(filtered);
		
    
    /*
    if(!lidar2base_transform) 
    {
      if(!tf_listener.canTransform(base_link_frame, cloud_msg->header.frame_id, ros::Time(0))) 
      {
        std::cerr << "failed to find transform between " << base_link_frame << " and " << cloud_msg->header.frame_id << std::endl;
        return;
      } 
      tf::StampedTransform transform;
      tf_listener.waitForTransform(base_link_frame, cloud_msg->header.frame_id, ros::Time(0), ros::Duration(2.0));
      tf_listener.lookupTransform(base_link_frame, cloud_msg->header.frame_id, ros::Time(0), transform);
      lidar2base_transform = transform;
    }
    
    if(lidar2base_transform)
		pcl_ros::transformPointCloud(*source_cloud, *source_cloud, *lidar2base_transform);
		
    */

    
	
	
	
	sensor_msgs::PointCloud2 out_cloud;
    pcl::toROSMsg(*filtered, out_cloud);
    out_cloud.header.stamp = cloud_msg->header.stamp;
    out_cloud.header.frame_id = base_link_frame;
    points_pub.publish(out_cloud);
  }

  pcl::PointCloud<PointT>::Ptr downsample(pcl::PointCloud<PointT>::Ptr cloud) const 
  {
    if(!downsample_filter) 
      return cloud;

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }

  pcl::PointCloud<PointT>::Ptr outlier_removal(pcl::PointCloud<PointT>::Ptr cloud) const 
  {
    if(!outlier_removal_filter) 
      return cloud;

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);

    return filtered;
  }
  
  //遠近高低濾波
  pcl::PointCloud<PointT>::Ptr distance_filter(const pcl::PointCloud<PointT>::ConstPtr cloud) const 
  {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());

    float near_thresh2 = distance_near_thresh*distance_near_thresh;
	float far_thresh2 = distance_far_thresh*distance_far_thresh;
	
    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points),
      [&](const PointT& p) 
      {  //dis = p.getVector3fMap().norm();
        float dis2 = p.x*p.x+p.y*p.y+p.z*p.z;
        return dis2 > near_thresh2 && dis2 < far_thresh2 && p.z < height_thresh ;
      });

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    return filtered;
  }
  

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Subscriber points_sub;
  ros::Publisher points_pub;

  tf::TransformListener tf_listener;
  boost::optional<tf::StampedTransform> lidar2base_transform;

  std::string raw_points_topic;
  
  std::string base_link_frame;

  bool use_distance_filter;
  double distance_near_thresh;
  double distance_far_thresh;
  double height_thresh;

  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Filter<PointT>::Ptr outlier_removal_filter;

  bool use_outlier_filter;
};

}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::PrefilteringNodelet, nodelet::Nodelet)
