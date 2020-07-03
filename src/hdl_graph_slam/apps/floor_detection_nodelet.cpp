#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/point_cloud.h>

#include <std_msgs/Time.h>
#include <sensor_msgs/PointCloud2.h>
#include <hdl_graph_slam/FloorCoeffs.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace hdl_graph_slam {

class FloorDetectionNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FloorDetectionNodelet() {}
  virtual ~FloorDetectionNodelet() {}

  virtual void onInit() {
    NODELET_DEBUG("initializing floor_detection_nodelet...");
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    points_sub = nh.subscribe(points_topic , 256, &FloorDetectionNodelet::cloud_callback, this);
    floor_pub = nh.advertise<hdl_graph_slam::FloorCoeffs>("/floor_detection/floor_coeffs", 32);

    read_until_pub = nh.advertise<std_msgs::Header>("/floor_detection/read_until", 32);
    floor_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/floor_detection/floor_filtered_points", 32);
    floor_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/floor_detection/floor_points", 32);
  }

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    tilt_deg = private_nh.param<double>("tilt_deg", 0.0);                          // approximate sensor tilt angle [deg]
    sensor_height = private_nh.param<double>("sensor_height", 2.0);                // approximate sensor height [m]
    height_clip_range= private_nh.param<double>("height_clip_range", 1.0);         // points with heights in [sensor_height - height_clip_range, sensor_height + height_clip_range] will be used for floor detection
    floor_pts_thresh = private_nh.param<int>("floor_pts_thresh", 512);             // minimum number of support points of RANSAC to accept a detected floor plane
    floor_normal_thresh = private_nh.param<double>("floor_normal_thresh", 10.0);   // verticality check thresold for the detected floor plane [deg]
    use_normal_filtering = private_nh.param<bool>("use_normal_filtering", true);   // if true, points with "non-"vertical normals will be filtered before RANSAC
    normal_filter_thresh = private_nh.param<double>("normal_filter_thresh", 20.0); // "non-"verticality check threshold [deg]

    points_topic = private_nh.param<std::string>("points_topic", "/velodyne_points");
    if(tilt_deg == 0)
      use_tilt_compensate = false;
    else
      use_tilt_compensate = true;
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
  {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(cloud->empty()) 
    {
      ROS_INFO("cloud is empty!");
      return;
    }
	
    // floor detection
    boost::optional<Eigen::Vector4f> floor = detect(cloud);
	
    // publish the detected floor coefficients
    hdl_graph_slam::FloorCoeffs coeffs;
    coeffs.header = cloud_msg->header;
    if(floor) 
    {
      coeffs.coeffs.resize(4);
      for(int i=0; i<4; i++) 
        coeffs.coeffs[i] = (*floor)[i];
    }

    floor_pub.publish(coeffs);

    // for offline estimation
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = points_topic;
    read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);
  }

  /**
   * @brief detect the floor plane from a point cloud
   * @param cloud  input cloud
   * @return detected floor plane coefficients
   * 利用平面模型进行地面检测，
   * 利用上次地面的检测结果分割当前地面点范围
   */
  boost::optional<Eigen::Vector4f> detect(const pcl::PointCloud<PointT>::Ptr& cloud) const 
  {
    static Eigen::Vector4f last_floor_coffs; static bool last_floor_coffs_valid = false;
	pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
	static Eigen::Matrix4f tilt_matrix;
	if(use_tilt_compensate)  //补偿雷达安装倾斜角？点云降采样时已经根据雷达的安装位置将点云转换至base_link,再次补偿无益！
	{        
		// compensate the tilt rotation
		tilt_matrix.setIdentity();

		//Eigen::Vector3f::UnitY() Y轴单位向量 [0,1,0].T
		// Eigen::AngleAxisf(tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix() 
		//旋转矩阵(沿向量轴旋转一定的角度)
		tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();
		// filtering before RANSAC (height and normal filtering)
		pcl::transformPointCloud(*cloud, *filtered, tilt_matrix);
		//平面模型 Ax+By+Cz+D = 0
		filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + height_clip_range), false);
		filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height - height_clip_range), true);
    }
    else
    {
        if(last_floor_coffs_valid)
        {
            filtered = plane_clip(cloud,   last_floor_coffs + Eigen::Vector4f(0, 0, 0, height_clip_range), false);
			filtered = plane_clip(filtered,last_floor_coffs - Eigen::Vector4f(0, 0, 0, height_clip_range), true);
        }
        else
        {
		        //平面模型 Ax+By+Cz+D = 0
			filtered = plane_clip(cloud, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + height_clip_range), false);
			filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height - height_clip_range), true);
        }
    	
    }

    if(use_normal_filtering)  
      filtered = normal_filtering(filtered); //法线滤波，滤除法线方向超出阈值的点云

    if(use_tilt_compensate)
      pcl::transformPointCloud(*filtered, *filtered, static_cast<Eigen::Matrix4f>(tilt_matrix.inverse()));

    // too few points for RANSAC
    if(filtered->size() < floor_pts_thresh) {
      return boost::none;
    }

    // RANSAC
    pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(filtered));
    pcl::RandomSampleConsensus<PointT> ransac(model_p);
    ransac.setDistanceThreshold(0.1);     //距离阈值，距平面在此范围内的点均认为平面点
    ransac.computeModel();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ransac.getInliers(inliers->indices);

    // too few inliers
    if(inliers->indices.size() < floor_pts_thresh) {
      return boost::none;
    }

    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);

     // make the normal upward
    //if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f)
    if(coeffs[2] < 0)
      coeffs *= -1.0f;
	
    static float threshold = std::cos(floor_normal_thresh * M_PI / 180.0);
    //判断平面法线与z轴夹角，超出阈值则认为非法平面
    //cos(theta) = v1*v2/(|v1||v2|) => v1[2]
    if(coeffs[2] < threshold) 
    {
      last_floor_coffs_valid = false;
      // the normal is not vertical
      return boost::none;
    }

    if(floor_points_pub.getNumSubscribers()) 
    {
      pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>);
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(filtered);
      extract.setIndices(inliers);
      extract.filter(*inlier_cloud);
      inlier_cloud->header = cloud->header;

      floor_points_pub.publish(inlier_cloud);
    }
    
    if(floor_filtered_pub.getNumSubscribers()) 
    {
    	pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>);
    	inlier_cloud->header = cloud->header;
    	inlier_cloud->reserve(cloud->size());
    	double disCoffInvese = 1.0/sqrt(coeffs.head<3>().dot(coeffs.head<3>())); //1.0/sqrt(A*A+B*B+C*C)
    	copy_if(cloud->begin(),cloud->end(),std::back_inserter(inlier_cloud->points),
				[=](const PointT& point)
				{
					double dis = (coeffs[0]*point.x + coeffs[1]*point.y + coeffs[2]*point.z  +coeffs[3]) * disCoffInvese; //Ax+By+Cz+D
					return dis > 0.05;
				});
		//std::cout << inlier_cloud->size() << std::endl;
		floor_filtered_pub.publish(inlier_cloud);
    }
    last_floor_coffs_valid = true;
    last_floor_coffs = Eigen::Vector4f(coeffs);
    return last_floor_coffs;
  }

  /**
   * @brief plane_clip
   * @param src_cloud
   * @param plane
   * @param negative
   * @return
   */
  pcl::PointCloud<PointT>::Ptr plane_clip(const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative) const {
    pcl::PlaneClipper3D<PointT> clipper(plane);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    clipper.clipPointCloud3D(*src_cloud, indices->indices);

    pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*dst_cloud);

    return dst_cloud;
  }

  /**
   * @brief filter points with non-vertical normals
   * @param cloud  input cloud
   * @return filtered cloud
   */
  pcl::PointCloud<PointT>::Ptr normal_filtering(const pcl::PointCloud<PointT>::Ptr& cloud) const 
  {
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(10);   //邻域点个数，利用邻域点求取平面得到法线向量
    
    //预设法线的方向  因为法线方向有两种可能
    ne.setViewPoint(0.0f, 0.0f, sensor_height);
    ne.compute(*normals);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    filtered->reserve(cloud->size());

    static float threshold = std::cos(normal_filter_thresh * M_PI / 180.0);
    for (int i = 0; i < cloud->size(); i++) {
    	//.getNormalVector3fMap() 获取法线向量
    	//.normalized() 归一化向量
    	//.dot(Eigen::Vector3f::UnitZ() 取Z的值
      float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
      if (std::abs(dot) > threshold) {
        filtered->push_back(cloud->at(i));
      }
    }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    return filtered;
  }


private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // ROS topics
  ros::Subscriber points_sub;

  ros::Publisher floor_pub;
  ros::Publisher floor_points_pub;
  ros::Publisher floor_filtered_pub;

  std::string points_topic;
  ros::Publisher read_until_pub;

  // floor detection parameters
  // see initialize_params() for the details
  double tilt_deg;
  bool use_tilt_compensate;

  double sensor_height;
  double height_clip_range;

  int floor_pts_thresh;
  double floor_normal_thresh;

  bool use_normal_filtering;
  double normal_filter_thresh;
};

}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::FloorDetectionNodelet, nodelet::Nodelet)
