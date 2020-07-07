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

#include <tf/transform_broadcaster.h>
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
#include<geometry_msgs/Transform.h>
#include <eigen_conversions/eigen_msg.h>
#include<fstream>
#include <yaml.h>

/*
rosservice call /hdl_graph_slam/save_map "utm: false
resolution: 0.0
detination: '/home/zwei/wendao/slam/hdl_slam_ws/map.pcd'"

*/

#define __NAME__ "mapping_node"

namespace hdl_graph_slam {

class HdlGraphSlamNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::PointCloud2> MySyncPolicy1;
  HdlGraphSlamNodelet() {}
  virtual ~HdlGraphSlamNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    // Get the node handle with the Multi Threaded callback queue. (provides this nodelets custom remappings and name)
    // 多线程句柄
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    // init parameters
    map_frame_id = private_nh.param<std::string>("map_frame_id", "map");
    odom_frame_id = private_nh.param<std::string>("odom_frame_id", "odom");
    map_cloud_resolution = private_nh.param<double>("map_cloud_resolution", 0.05);
    trans_odom2map.setIdentity();
	  base_frame_id = private_nh.param<std::string>("base_frame_id", "base_link");
    max_keyframes_per_update = private_nh.param<int>("max_keyframes_per_update", 10);

    //
    anchor_node = nullptr;
    anchor_edge = nullptr;
    floor_plane_node = nullptr;
    graph_slam.reset(new GraphSLAM(private_nh.param<std::string>("g2o_solver_type", "lm_var")));
    keyframe_updater.reset(new KeyframeUpdater(private_nh));
    loop_detector.reset(new LoopDetector(private_nh));
    map_cloud_generator.reset(new MapCloudGenerator());
    inf_calclator.reset(new InformationMatrixCalculator(private_nh));
    nmea_parser.reset(new NmeaSentenceParser());

    //是否使用gps提供的odom，如果不使用，则使用点云配置所得odom
    use_gps_instead_lidar_odom = private_nh.param<bool>("use_gps_instead_lidar_odom", false);
    
    //gps
    gps_time_offset = private_nh.param<double>("gps_time_offset", 0.0);
    gps_edge_stddev_xy = private_nh.param<double>("gps_edge_stddev_xy", 10000.0);
    gps_edge_stddev_z = private_nh.param<double>("gps_edge_stddev_z", 10.0);
    
    //floor
    use_floor_edge = private_nh.param<bool>("use_floor_edge", true);
    floor_edge_stddev = private_nh.param<double>("floor_edge_stddev", 10.0);
    
    //utm
    enable_utm_xy = private_nh.param<bool>("enable_utm",false);
    utm_time_offset = private_nh.param<double>("utm_time_offset", 0.0);
    utm_edge_stddev_xy = private_nh.param<double>("utm_edge_stddev_xy", 10000.0);
    utm_edge_stddev_z = private_nh.param<double>("utm_edge_stddev_z", 10.0);


    imu_time_offset = private_nh.param<double>("imu_time_offset", 0.0);
    enable_imu_orientation = private_nh.param<bool>("enable_imu_orientation", false);
    enable_imu_acceleration = private_nh.param<bool>("enable_imu_acceleration", false);
    imu_orientation_edge_stddev = private_nh.param<double>("imu_orientation_edge_stddev", 0.1);
    imu_acceleration_edge_stddev = private_nh.param<double>("imu_acceleration_edge_stddev", 3.0);

    points_topic = private_nh.param<std::string>("points_topic", "/velodyne_poits");
  	std::string gps_odom_topic = private_nh.param<std::string>("gps_odom_topic","/gps_odom");
    // subscribers
    if(use_gps_instead_lidar_odom)
      odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, gps_odom_topic, 256));
    else
      odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, "/scan_matching_odometry/odom", 256));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, points_topic, 32));
    
    sync1.reset(new message_filters::Synchronizer<MySyncPolicy1>(MySyncPolicy1(10),*odom_sub, *cloud_sub));
	  sync1->registerCallback(boost::bind(&HdlGraphSlamNodelet::cloud_callback, this, _1, _2));
	
    imu_sub = nh.subscribe("/gpsimu_driver/imu_data", 1024, &HdlGraphSlamNodelet::imu_callback, this);
    floor_sub = nh.subscribe("/floor_detection/floor_coeffs", 1024, &HdlGraphSlamNodelet::floor_coeffs_callback, this);

    if(private_nh.param<bool>("enable_gps", false))
    {
      gps_sub = mt_nh.subscribe("/gps/geopoint", 1024, &HdlGraphSlamNodelet::gps_callback, this);
      nmea_sub = mt_nh.subscribe("/gpsimu_driver/nmea_sentence", 1024, &HdlGraphSlamNodelet::nmea_callback, this);
      navsat_sub = mt_nh.subscribe("/gps/navsat", 1024, &HdlGraphSlamNodelet::navsat_callback, this);
    }
    
    if((!use_gps_instead_lidar_odom && enable_utm_xy) || enable_imu_orientation)
    {
      std::string utm_topic = private_nh.param<std::string>("utm_topic","/gps_odom");
      utm_sub = mt_nh.subscribe(utm_topic,1024,&HdlGraphSlamNodelet::utm_callback, this);
    }

    // publishers
    markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/markers", 16);
    odom2map_pub = mt_nh.advertise<geometry_msgs::TransformStamped>("/hdl_graph_slam/odom2pub", 16);
    map_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/map_points", 1);
    transed_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/aligned_points", 1);
    read_until_pub = mt_nh.advertise<std_msgs::Header>("/hdl_graph_slam/read_until", 32);

    dump_service_server = mt_nh.advertiseService("/hdl_graph_slam/dump", &HdlGraphSlamNodelet::dump_service, this);
    save_map_service_server = mt_nh.advertiseService("/hdl_graph_slam/save_map", &HdlGraphSlamNodelet::save_map_service, this);
    save_odom_service_server = mt_nh.advertiseService("/hdl_graph_slam/save_odom",&HdlGraphSlamNodelet::save_odom_service,this);

    double graph_update_interval = private_nh.param<double>("graph_update_interval", 3.0);
    double map_cloud_update_interval = private_nh.param<double>("map_cloud_update_interval", 10.0);
    
    //ros::Timer使用ROS Clock
    //如果想要定时器使用wall-clock时间，可以替代Timer为WallTimer
    optimization_timer = mt_nh.createWallTimer(ros::WallDuration(graph_update_interval), &HdlGraphSlamNodelet::optimization_timer_callback, this);
    map_publish_timer = mt_nh.createWallTimer(ros::WallDuration(map_cloud_update_interval), &HdlGraphSlamNodelet::map_points_publish_timer_callback, this);
  }
private:
  /*@brief 将gps全局定位信息转换为车辆相对于地图的局部定位信息
   *@bried 期望输出base_link相对于odom的坐标.
   *@brief 实际输出为base_link相对于map的坐标
   *@brief 当odom与map重合时,结果正确,当不重合时可能出现错误！
   */
  Eigen::Isometry3d get_odom_by_gps(const nav_msgs::OdometryConstPtr& utm_odom_msg)
  {
    nav_msgs::Odometry odom = *utm_odom_msg;
    odom.pose.pose.position.z = 0;
    
    if(!gps_to_base)
    {
      //获取gps与base_link的坐标变换
      std::string gps_frame_id = utm_odom_msg->child_frame_id;
      if(gps_frame_id.empty())
      {
          gps_frame_id = "gps";
          ROS_INFO("gps odom child frame id is empty, set to gps");
      }
      
      tf::StampedTransform tf_gps_in_base;
      try
      {
        tf_listener.waitForTransform(base_frame_id ,gps_frame_id, ros::Time(0), ros::Duration(1.0));
        tf_listener.lookupTransform(base_frame_id , gps_frame_id, ros::Time(0), tf_gps_in_base);
      } 
      catch (std::exception& e) 
      {
        std::cerr << "failed to find the transform from [" << base_frame_id << "] to [" << utm_odom_msg->child_frame_id << "]!!" << std::endl;
        return Eigen::Isometry3d::Identity();
      }
      
      Eigen::Isometry3d _gps_to_base;
      tf::transformTFToEigen(tf_gps_in_base.inverse(), _gps_to_base);
      
      gps_to_base = _gps_to_base;
    }
    //由gps定位点与gps在base_link的安装位置,求base_link的大地坐标
    Eigen::Isometry3d gps_in_world = odom2isometry(odom);
    Eigen::Isometry3d base_in_world =  gps_in_world * (*gps_to_base);
    
//    std::cout << "gps_in_world : " << gps_in_world.translation().transpose() << std::endl;
//    std::cout << "base_in_world : " << base_in_world.translation().transpose() << std::endl;
    
    //地图原点在大地坐标系下的位置
    if(!map_in_world)
      map_in_world = base_in_world;
      
    static Eigen::Isometry3d map_in_world_reverse = map_in_world->inverse();
    
    //base_in_map -> local_odom
    return map_in_world_reverse * base_in_world;
  }
  
  
  
  Eigen::Quaterniond get_orientation_by_gps(const nav_msgs::OdometryConstPtr& utm_odom_msg)
  {
    const geometry_msgs::Quaternion& gps_ori_msg = utm_odom_msg->pose.pose.orientation;
    Eigen::Quaterniond gps_ori(gps_ori_msg.w, gps_ori_msg.x, gps_ori_msg.y, gps_ori_msg.z);
    gps_ori.normalize();
    
    if(!gps_to_base)
    {
      //获取gps与base_link的坐标变换
      std::string gps_frame_id = utm_odom_msg->child_frame_id;
      if(gps_frame_id.empty())
      {
          gps_frame_id = "gps";
          ROS_INFO("gps odom child frame id is empty, set to gps");
      }
      
      tf::StampedTransform tf_gps2base;
      try
      {
        tf_listener.waitForTransform(base_frame_id ,gps_frame_id, ros::Time(0), ros::Duration(1.0));
        tf_listener.lookupTransform(base_frame_id , gps_frame_id, ros::Time(0), tf_gps2base);
      } 
      catch (std::exception& e) 
      {
        std::cerr << "failed to find the transform from [" << base_frame_id << "] to [" << utm_odom_msg->child_frame_id << "]!!" << std::endl;
        return gps_ori;
      }
      
      Eigen::Isometry3d _gps_to_base_;
      tf::transformTFToEigen(tf_gps2base, _gps_to_base_);
      
      gps_to_base = _gps_to_base_;
    }
    
    Eigen::Matrix3d gps_rot_matrix = gps_ori.matrix();  //gps在世界坐标系下的旋转矩阵
    Eigen::Matrix3d base_rot_matrix = (*gps_to_base).linear() * gps_rot_matrix; //base_link在世界坐标系下的旋转矩阵
    static Eigen::Matrix3d origin_base_rot_matrix_inv = base_rot_matrix.inverse();//base_link位于地图原点时，在世界坐标系下的旋转矩阵，的逆
    
    Eigen::Quaterniond result(origin_base_rot_matrix_inv * base_rot_matrix); //base_link在地图map坐标系下的旋转矩阵
    return result;
  }

  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @brief 接收点云以及筛选关键帧(第一帧点云为关键帧，与上一帧点云变换关系超出阈值的为当前关键帧)
   * @param odom_msg
   * @param cloud_msg
   */
  void cloud_callback(const nav_msgs::OdometryConstPtr& odom_msg,
					  const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) 
  {
	//ROS_INFO("slam received a pointcloud");

    Eigen::Isometry3d odom;
    if(use_gps_instead_lidar_odom)
    {
        odom = get_odom_by_gps(odom_msg);
        geometry_msgs::TransformStamped tf_odom = matrix2transform(odom_msg->header.stamp, odom, odom_frame_id, base_frame_id);
        odom_by_gps_broadcaster.sendTransform(tf_odom);
    }
    else
      odom = odom2isometry(odom_msg);
      
    const ros::Time& stamp = cloud_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    if(transed_points_pub.getNumSubscribers())
    {
		// 发布转换到odom坐标系下的点云
		Eigen::Matrix4f tran_ = odom.matrix().cast<float>();
		pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
		pcl::transformPointCloud(*cloud, *aligned, tran_);
		sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
		cloud_msg->header.stamp = cloud_msg->header.stamp;
		cloud_msg->header.frame_id = odom_frame_id;
		pcl::toROSMsg(*aligned, *cloud_msg);
		transed_points_pub.publish(aligned);
    }

		//前后帧变换足够大时，更新累计路程，
		//并记录当前帧位姿,当前帧为关键帧
    if(!keyframe_updater->update(odom))  //与上一关键帧位置对比，变换是否超出阈值
    {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      if(keyframe_queue.empty() && read_until_pub.getNumSubscribers()) 
      {
        std_msgs::Header read_until;
        read_until.stamp = stamp + ros::Duration(10, 0);
        read_until.frame_id = points_topic;
        read_until_pub.publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub.publish(read_until);
      }
      return;
    }

    double accum_d = keyframe_updater->get_accum_distance();
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.push_back(keyframe);
  }

// keyframe_queue 所有接收到的点云关键帧先存入此队列
// keyframes 保存所有关键帧
// new_keyframes 将keyframe_queue中的一批关键帧存入new_keyframes，然后与keyframes对比进行回环检测，之后将其添加进keyframes

// keyframes_snapshot 
// keyframe_hash 关键帧哈希表，以关键帧时间为键，可用地面平面模型数据时间戳查找对应关键帧
// 

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
   * @brief 添加keyframe_queue中的关键帧里程计pose到图优化`节点`
   * @brief 添加前后帧之间的相对姿态到图优化中的`边`
   * @return if true, at least one keyframe was added to the pose graph
   */
  bool flush_keyframe_queue()
  {
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

    if(keyframe_queue.empty()) 
      return false;

    trans_odom2map_mutex.lock();
    //odom2map 初始为I(即两者重合),后经优化器更新，反映里程计误差
    Eigen::Isometry3d odom2map(trans_odom2map.cast<double>()); 
    trans_odom2map_mutex.unlock();

    int num_processed = 0;
    for(int i=0; i<std::min<int>(keyframe_queue.size(), max_keyframes_per_update); ++i) 
    {
      num_processed = i;

      const auto& keyframe = keyframe_queue[i];
      // new_keyframes will be tested later for loop closure
      // new_keyframes 将被用于回环检测
      new_keyframes.push_back(keyframe);

      // add pose node 添加姿态节点
      Eigen::Isometry3d odom = odom2map * keyframe->odom;
      keyframe->node = graph_slam->add_se3_node(odom);
      keyframe_hash[keyframe->stamp] = keyframe;

      // fix the first node
      if(keyframes.empty() && new_keyframes.size() == 1) {
        if(private_nh.param<bool>("fix_first_node", false)) {
          anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());
          anchor_node->setFixed(true);
          anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 6));
        }
      }

      if(i==0 && keyframes.empty()) //i为0且keyframes为空时，不存在上一关键帧
        continue;

      // add edge between consecutive keyframes
      const auto& prev_keyframe = (i==0) ? keyframes.back():keyframe_queue[i - 1];
      //与上一关键帧相对当前帧的变换
      Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
      // 计算信息矩阵
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(prev_keyframe->cloud, keyframe->cloud, relative_pose);
      auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
      static std::string robust_kernel = private_nh.param<std::string>("odometry_edge_robust_kernel", "NONE");
      static double robust_kernel_size = private_nh.param<double>("odometry_edge_robust_kernel_size", 1.0);
      graph_slam->add_robust_kernel(edge, robust_kernel, robust_kernel_size);
    }

    std_msgs::Header read_until;
    read_until.stamp = keyframe_queue[num_processed]->stamp + ros::Duration(10, 0);
    read_until.frame_id = points_topic;
    read_until_pub.publish(read_until);
    read_until.frame_id = "/filtered_points";
    read_until_pub.publish(read_until);

    keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);
    return true;
  }

  void nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) {
    GPRMC grmc = nmea_parser->parse(nmea_msg->sentence);

    if(grmc.status != 'A') {
      return;
    }

    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg->header = nmea_msg->header;
    gps_msg->position.latitude = grmc.latitude;
    gps_msg->position.longitude = grmc.longitude;
    gps_msg->position.altitude = NAN;

    gps_callback(gps_msg);
  }

  void navsat_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg->header = navsat_msg->header;
    gps_msg->position.latitude = navsat_msg->latitude;
    gps_msg->position.longitude = navsat_msg->longitude;
    gps_msg->position.altitude = navsat_msg->altitude;
    gps_callback(gps_msg);
  }

  /**
   * @brief received gps data is added to #gps_queue
   * @param gps_msg
   */
  void gps_callback(const geographic_msgs::GeoPointStampedPtr& gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    gps_msg->header.stamp += ros::Duration(gps_time_offset);
    gps_queue.push_back(gps_msg);
  }

  /**
   * @brief
   * @return
   */
  bool flush_gps_queue() 
  {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);

    if(keyframes.empty() || gps_queue.empty())
      return false;

    bool updated = false;
    auto gps_cursor = gps_queue.begin();

    for(auto& keyframe : keyframes) 
    {
      if(keyframe->stamp > gps_queue.back()->header.stamp)
        break;
      
      //1.keyframe时间比第一帧gps时间还早
      //2.当前keyframe 的 utm_coord 已经被赋值
      if(keyframe->stamp < (*gps_cursor)->header.stamp || keyframe->utm_coord)
        continue;

      // find the gps data which is closest to the keyframe
      // 查找与关键帧时间最近的gps数据
      auto closest_gps = gps_cursor;
      for(auto gps = gps_cursor; gps != gps_queue.end(); ++gps) 
      {
        auto dt = ((*closest_gps)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*gps)->header.stamp - keyframe->stamp).toSec();
        if(std::abs(dt) < std::abs(dt2))
          break;

        closest_gps = gps;
      }

      // if the time residual between the gps and keyframe is too large, skip it
      // 如果GPS数据时间与关键帧时间相差太大则略去
      gps_cursor = closest_gps;
      if(0.2 < std::abs(((*closest_gps)->header.stamp - keyframe->stamp).toSec()))
        continue;

      // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
      // 将经纬度转换为utm
      geodesy::UTMPoint utm;
      geodesy::fromMsg((*closest_gps)->position, utm);
      Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

      // the first gps data position will be the origin of the map
      // 将第一帧有效GPS位置作为地图的原点
      if(!zero_utm)
        zero_utm = xyz;
        
      xyz -= (*zero_utm);

      keyframe->utm_coord = xyz;

      g2o::OptimizableGraph::Edge* edge;
      if(std::isnan(xyz.z()))
      {
        Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
        edge = graph_slam->add_se3_prior_xy_edge(keyframe->node, xyz.head<2>(), information_matrix);
      } 
      else 
      {
        Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
        information_matrix.block<2, 2>(0, 0) /= gps_edge_stddev_xy;
        information_matrix(2, 2) /= gps_edge_stddev_z;
        edge = graph_slam->add_se3_prior_xyz_edge(keyframe->node, xyz, information_matrix);
      }
      
      static std::string gps_edge_robust_kernel = private_nh.param<std::string>("gps_edge_robust_kernel", "NONE");
      static double gps_edge_robust_kernel_size = private_nh.param<double>("gps_edge_robust_kernel_size", 1.0);
      graph_slam->add_robust_kernel(edge, gps_edge_robust_kernel, gps_edge_robust_kernel_size);

      updated = true;
    }

    auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), keyframes.back()->stamp,
      [=](const ros::Time& stamp, const geographic_msgs::GeoPointStampedConstPtr& geopoint) {
        return stamp < geopoint->header.stamp;});
    gps_queue.erase(gps_queue.begin(), remove_loc);
    return updated;
  }
  
  void utm_callback(const nav_msgs::Odometry::Ptr& utm_msg)
  {
  	std::lock_guard<std::mutex> lock(utm_queue_mutex);
  	utm_msg->header.stamp += ros::Duration(utm_time_offset);
    utm_queue.push_back(utm_msg);
  }
  
  bool flush_utm_queue()
  {
    std::lock_guard<std::mutex> lock(utm_queue_mutex);
    if(keyframes.empty() || utm_queue.empty())
      return false;

    bool updated = false;
    auto utm_cursor = utm_queue.begin();

    for(auto& keyframe : keyframes) 
    {
      if(keyframe->stamp > utm_queue.back()->header.stamp)
        break;
      
      //1.keyframe时间比第一帧gps时间还早
      //2.当前keyframe 的 utm_coord 已经被赋值
      if(keyframe->stamp < (*utm_cursor)->header.stamp || keyframe->utm_coord)
        continue;

      // find the utm data which is closest to the keyframe
      // 循环前后帧对比，查找距离关键帧最近的utm数据
      auto closest_utm = utm_cursor;
      for(auto utm = utm_cursor; utm != utm_queue.end(); utm++) 
      {
        auto dt = ((*closest_utm)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*utm)->header.stamp - keyframe->stamp).toSec();
        if(std::abs(dt) < std::abs(dt2))
          break;

        closest_utm = utm;
      }

      // if the time residual between the utm and keyframe is too large, skip it
      if(0.2 < std::abs(((*closest_utm)->header.stamp - keyframe->stamp).toSec())) 
      {
          ROS_INFO("[%s] the time residual between the utm and keyframe is too large, skip it.",__NAME__);
          continue;
      }
      
      if(!zero_utm)
      {
          const auto &pose = (*closest_utm)->pose.pose.position;
          Eigen::Vector3d global_xyz(pose.x, pose.y, 0);
          zero_utm = global_xyz;
      }
      if(!zero_orientation)
      {
          const auto& ros_quat = (*closest_utm)->pose.pose.orientation;
      
          Eigen::Quaterniond quat(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z);
          zero_orientation = quat;
      }
         
      if(enable_utm_xy && !keyframe->utm_coord)
      {
          Eigen::Isometry3d utm_odom = get_odom_by_gps(*closest_utm);
		  Eigen::Vector3d local_xyz = utm_odom.translation(); local_xyz[2] = 0;
		  
		  keyframe->utm_coord = local_xyz;
		  
		  std::cout << "keyframe position by utm " << local_xyz.transpose() << "\n"; 
		
		  g2o::OptimizableGraph::Edge* edge;
		  
		  Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / utm_edge_stddev_xy;
		  edge = graph_slam->add_se3_prior_xy_edge(keyframe->node, local_xyz.head<2>(), information_matrix);
		  
		  static std::string utm_edge_robust_kernel = private_nh.param<std::string>("utm_edge_robust_kernel", "NONE");
		  static double utm_edge_robust_kernel_size = private_nh.param<double>("utm_edge_robust_kernel_size", 1.0);
		  graph_slam->add_robust_kernel(edge, utm_edge_robust_kernel, utm_edge_robust_kernel_size);
	  }

      if(enable_imu_orientation && !keyframe->orientation)
      {
        keyframe->orientation = get_orientation_by_gps(*closest_utm);
        if(keyframe->orientation->w() < 0.0)
           keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_orientation_edge_stddev; //yaw roll pitch
        //info(0,0) = info(1,1) = info(2,2) = 0.01;  //弱化权重
        
        auto edge = graph_slam->add_se3_prior_quat_edge(keyframe->node, *keyframe->orientation, info);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_orientation_edge_robust_kernel", "NONE"), 
        									private_nh.param<double>("imu_orientation_edge_robust_kernel_size", 1.0));
      }

      updated = true;
    }
    
    
    // 移除陈旧的utm数据
    auto remove_loc = std::upper_bound(utm_queue.begin(), utm_queue.end(), keyframes.back()->stamp,
      [=](const ros::Time& stamp, const nav_msgs::Odometry::Ptr& utm_msg) {
        return stamp < utm_msg->header.stamp;});
        
    utm_queue.erase(utm_queue.begin(), remove_loc);
    return updated;
  }
  

  void imu_callback(const sensor_msgs::ImuPtr& imu_msg) 
  {
    if(!enable_imu_orientation && !enable_imu_acceleration)
      return;

    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    imu_msg->header.stamp += ros::Duration(imu_time_offset);
    imu_queue.push_back(imu_msg);
  }

  bool flush_imu_queue() {
    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    if(keyframes.empty() || imu_queue.empty() || base_frame_id.empty())
      return false;

    bool updated = false;
    auto imu_cursor = imu_queue.begin();

    for(auto& keyframe : keyframes) 
    {
      if(keyframe->stamp > imu_queue.back()->header.stamp)
        break;

      if(keyframe->stamp < (*imu_cursor)->header.stamp || keyframe->acceleration || keyframe->orientation)
        continue;

      // find imu data which is closest to the keyframe
      auto closest_imu = imu_cursor;
      for(auto imu = imu_cursor; imu != imu_queue.end(); imu++) 
      {
        auto dt = ((*closest_imu)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*imu)->header.stamp - keyframe->stamp).toSec();
        if(std::abs(dt) < std::abs(dt2))
          break;

        closest_imu = imu;
      }

      imu_cursor = closest_imu;
      if(0.2 < std::abs(((*closest_imu)->header.stamp - keyframe->stamp).toSec()))
        continue;

      const auto& imu_ori = (*closest_imu)->orientation;
      const auto& imu_acc = (*closest_imu)->linear_acceleration;

      geometry_msgs::Vector3Stamped acc_imu;
      geometry_msgs::Vector3Stamped acc_base;
      geometry_msgs::QuaternionStamped quat_imu;
      geometry_msgs::QuaternionStamped quat_base;

      quat_imu.header.frame_id = acc_imu.header.frame_id = (*closest_imu)->header.frame_id;
      quat_imu.header.stamp = acc_imu.header.stamp = ros::Time(0);
      acc_imu.vector = (*closest_imu)->linear_acceleration;
      quat_imu.quaternion = (*closest_imu)->orientation;

      try 
      {
        tf_listener.transformVector(base_frame_id, acc_imu, acc_base);
        tf_listener.transformQuaternion(base_frame_id, quat_imu, quat_base);
      } 
      catch (std::exception& e) 
      {
        std::cerr << "failed to find transform!!" << std::endl;
        return false;
      }

      keyframe->acceleration = Eigen::Vector3d(acc_base.vector.x, acc_base.vector.y, acc_base.vector.z);
      keyframe->orientation = Eigen::Quaterniond(quat_base.quaternion.w, quat_base.quaternion.x, quat_base.quaternion.y, quat_base.quaternion.z);
      //keyframe->orientation = keyframe->orientation;
      if(keyframe->orientation->w() < 0.0) {
        keyframe->orientation->coeffs() = -keyframe->orientation->coeffs();
      }

      if(enable_imu_orientation) 
      {
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_orientation_edge_stddev;
        auto edge = graph_slam->add_se3_prior_quat_edge(keyframe->node, *keyframe->orientation, info);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_orientation_edge_robust_kernel", "NONE"), 
        									private_nh.param<double>("imu_orientation_edge_robust_kernel_size", 1.0));
      }

      if(enable_imu_acceleration) 
      {
        Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_acceleration_edge_stddev;
        g2o::OptimizableGraph::Edge* edge = graph_slam->add_se3_prior_vec_edge(keyframe->node, Eigen::Vector3d::UnitZ(), *keyframe->acceleration, info);
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_acceleration_edge_robust_kernel", "NONE"), 
        									private_nh.param<double>("imu_acceleration_edge_robust_kernel_size", 1.0));
      }
      updated = true;
    }

    auto remove_loc = std::upper_bound(imu_queue.begin(), imu_queue.end(), keyframes.back()->stamp,
      [=](const ros::Time& stamp, const sensor_msgs::ImuConstPtr& imu) {
        return stamp < imu->header.stamp;});
    imu_queue.erase(imu_queue.begin(), remove_loc);

    return true;
  }


  /**
   * @brief received floor coefficients are added to #floor_coeffs_queue
   * @param floor_coeffs_msg
   */
  void floor_coeffs_callback(const hdl_graph_slam::FloorCoeffsConstPtr& floor_coeffs_msg) 
  {
    if(floor_coeffs_msg->coeffs.empty()) 
      return;

    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);
    floor_coeffs_queue.push_back(floor_coeffs_msg);
  }

  /**
   * @brief this methods associates floor coefficients messages with registered keyframes, and then adds the associated coeffs to the pose graph
   * @return if true, at least one floor plane edge is added to the pose graph
   */
  bool flush_floor_queue() 
  {
    if(!use_floor_edge)
        return true;
        
    std::lock_guard<std::mutex> lock(floor_coeffs_queue_mutex);

    if(keyframes.empty()) 
      return false;

    const auto& latest_keyframe_stamp = keyframes.back()->stamp;

    bool updated = false;
    for(const auto& floor_coeffs : floor_coeffs_queue) {
      if(floor_coeffs->header.stamp > latest_keyframe_stamp) {
        break;
      }

      auto found = keyframe_hash.find(floor_coeffs->header.stamp);
      if(found == keyframe_hash.end()) {
        continue;
      }

      if(!floor_plane_node) 
      {
        floor_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
        floor_plane_node->setFixed(true);
      }

      const auto& keyframe = found->second;

      Eigen::Vector4d coeffs(floor_coeffs->coeffs[0], floor_coeffs->coeffs[1], floor_coeffs->coeffs[2], floor_coeffs->coeffs[3]);
      Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
      auto edge = graph_slam->add_se3_plane_edge(keyframe->node, floor_plane_node, coeffs, information);
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("floor_edge_robust_kernel", "NONE"), private_nh.param<double>("floor_edge_robust_kernel_size", 1.0));

      keyframe->floor_coeffs = coeffs;

      updated = true;
    }

    auto remove_loc = std::upper_bound(floor_coeffs_queue.begin(), floor_coeffs_queue.end(), latest_keyframe_stamp,
      [=](const ros::Time& stamp, const hdl_graph_slam::FloorCoeffsConstPtr& coeffs) {
        return stamp < coeffs->header.stamp;
      }
    );
    floor_coeffs_queue.erase(floor_coeffs_queue.begin(), remove_loc);

    return updated;
  }

  /**
   * @brief generate map point cloud and publish it
   * 生成地图点并发布，所有点均经坐标变换为优化后的位置，计算量巨大
   * @param event
   */
  void map_points_publish_timer_callback(const ros::WallTimerEvent& event) {
    if(!map_points_pub.getNumSubscribers()) {
      return;
    }

    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, 0.05);
    if(!cloud) {
      return;
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);

    map_points_pub.publish(cloud_msg);
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   * @param event
   */
  void optimization_timer_callback(const ros::WallTimerEvent& event) 
  {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    // add keyframes and floor coeffs in the queues to the pose graph
    bool keyframe_updated = flush_keyframe_queue();

    if(!keyframe_updated) 
    {
      std_msgs::Header read_until;
      read_until.stamp = ros::Time::now() + ros::Duration(30, 0);
      read_until.frame_id = points_topic;
      read_until_pub.publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub.publish(read_until);
    }

    if(!keyframe_updated & !flush_floor_queue() & !flush_gps_queue() &!flush_imu_queue() &!flush_utm_queue()) 
      return;

    // loop detection 回环检测
    std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam);
    std::cout << "loops size: " << loops.size() <<std::endl;
    for(const auto& loop : loops) 
    {
      Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
      Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);
      auto edge = graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);

      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("loop_closure_edge_robust_kernel", "NONE"), private_nh.param<double>("loop_closure_edge_robust_kernel_size", 1.0));
    }

    //将使用过的新关键帧集合添加至关键帧集合后清空
    std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
    new_keyframes.clear();

    // optimize the pose graph
    static int num_iterations = private_nh.param<int>("g2o_solver_num_iterations", 1024);
    graph_slam->optimize(num_iterations);

    // publish tf
    const auto& keyframe = keyframes.back();
    //trans：最后一个关键帧位置的估计值与观测值之差
    Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
    trans_odom2map_mutex.lock();
    //估计值与观测值的差异即为odom坐标系到map坐标系的差异
    trans_odom2map = trans.matrix().cast<float>();
    trans_odom2map_mutex.unlock();

    // 建立临时关键帧快照，然后再通过swap交换于关键帧快照，避免快照建立时长时间加锁
    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
    std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(),
      [=](const KeyFrame::Ptr& k) {
        return std::make_shared<KeyFrameSnapshot>(k);
    });

    keyframes_snapshot_mutex.lock();
    keyframes_snapshot.swap(snapshot);
    keyframes_snapshot_mutex.unlock();

    if(odom2map_pub.getNumSubscribers()) {
      geometry_msgs::TransformStamped ts = matrix2transform(keyframe->stamp, trans.matrix().cast<float>(), map_frame_id, odom_frame_id);
      odom2map_pub.publish(ts);
    }

    if(markers_pub.getNumSubscribers()) {
      auto markers = create_marker_array(ros::Time::now());
      markers_pub.publish(markers);
    }
  }

  /**
   * @brief create visualization marker
   * @param stamp
   * @return
   */
  visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp) const {
    visualization_msgs::MarkerArray markers;
    markers.markers.resize(5);

    // node markers
    visualization_msgs::Marker& traj_marker = markers.markers[0];
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;

    visualization_msgs::Marker& imu_marker = markers.markers[4];
    imu_marker.header = traj_marker.header;
    imu_marker.ns = "imu";
    imu_marker.id = 4;
    imu_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    imu_marker.pose.orientation.w = 1.0;
    imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.75;

    traj_marker.points.resize(keyframes.size());
    traj_marker.colors.resize(keyframes.size());
    for(int i=0; i<keyframes.size(); i++) {
      Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
      traj_marker.points[i].x = pos.x();
      traj_marker.points[i].y = pos.y();
      traj_marker.points[i].z = pos.z();

      double p = static_cast<double>(i) / keyframes.size();
      traj_marker.colors[i].r = 1.0 - p;
      traj_marker.colors[i].g = p;
      traj_marker.colors[i].b = 0.0;
      traj_marker.colors[i].a = 1.0;

      if(keyframes[i]->acceleration) {
        Eigen::Vector3d pos = keyframes[i]->node->estimate().translation();
        geometry_msgs::Point point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();

        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 0.1;

        imu_marker.points.push_back(point);
        imu_marker.colors.push_back(color);
      }
    }

    // edge markers
    visualization_msgs::Marker& edge_marker = markers.markers[1];
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = 1;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;

    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.05;

    edge_marker.points.resize(graph_slam->graph->edges().size() * 2);
    edge_marker.colors.resize(graph_slam->graph->edges().size() * 2);

    auto edge_itr = graph_slam->graph->edges().begin();
    for(int i=0; edge_itr != graph_slam->graph->edges().end(); edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *edge_itr;
      g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
      if(edge_se3) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = v2->estimate().translation();

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z();
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();

        double p1 = static_cast<double>(v1->id()) / graph_slam->graph->vertices().size();
        double p2 = static_cast<double>(v2->id()) / graph_slam->graph->vertices().size();
        edge_marker.colors[i*2].r = 1.0 - p1;
        edge_marker.colors[i*2].g = p1;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 1.0 - p2;
        edge_marker.colors[i*2 + 1].g = p2;
        edge_marker.colors[i*2 + 1].a = 1.0;

        if(std::abs(v1->id() - v2->id()) > 2) {
          edge_marker.points[i*2].z += 0.5;
          edge_marker.points[i*2 + 1].z += 0.5;
        }

        continue;
      }

      g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);
      if(edge_plane) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_plane->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2(pt1.x(), pt1.y(), 0.0);

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z();
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();

        edge_marker.colors[i*2].b = 1.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].b = 1.0;
        edge_marker.colors[i*2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXY* edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY*>(edge);
      if(edge_priori_xy) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xy->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
        pt2.head<2>() = edge_priori_xy->measurement();

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z() + 0.5;
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z() + 0.5;

        edge_marker.colors[i*2].r = 1.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 1.0;
        edge_marker.colors[i*2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXYZ* edge_priori_xyz = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge);
      if(edge_priori_xyz) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xyz->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = edge_priori_xyz->measurement();

        edge_marker.points[i*2].x = pt1.x();
        edge_marker.points[i*2].y = pt1.y();
        edge_marker.points[i*2].z = pt1.z() + 0.5;
        edge_marker.points[i*2 + 1].x = pt2.x();
        edge_marker.points[i*2 + 1].y = pt2.y();
        edge_marker.points[i*2 + 1].z = pt2.z();

        edge_marker.colors[i*2].r = 1.0;
        edge_marker.colors[i*2].a = 1.0;
        edge_marker.colors[i*2 + 1].r = 1.0;
        edge_marker.colors[i*2 + 1].a = 1.0;

        continue;
      }
    }

    // sphere
    visualization_msgs::Marker& sphere_marker = markers.markers[3];
    sphere_marker.header.frame_id = "map";
    sphere_marker.header.stamp = stamp;
    sphere_marker.ns = "loop_close_radius";
    sphere_marker.id = 0;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;

    if(!keyframes.empty()) {
      Eigen::Vector3d pos = keyframes.back()->node->estimate().translation();
      sphere_marker.pose.position.x = pos.x();
      sphere_marker.pose.position.y = pos.y();
      sphere_marker.pose.position.z = pos.z();
    }
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector->get_distance_thresh() * 2.0;

    sphere_marker.color.r = 1.0;
    sphere_marker.color.a = 0.3;

    return markers;
  }

  /**
   * @brief dump all data to the current directory
   * @param req
   * @param res
   * @return
   */
  bool dump_service(hdl_graph_slam::DumpGraphRequest& req, hdl_graph_slam::DumpGraphResponse& res) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    std::string directory = req.destination;

    if(directory.empty()) {
      std::array<char, 64> buffer;
      buffer.fill(0);
      time_t rawtime;
      time(&rawtime);
      const auto timeinfo = localtime(&rawtime);
      strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
      std::string directory(buffer.data());
    }

    if(!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }

    std::cout << "all data dumped to:" << directory << std::endl;

    graph_slam->save(directory + "/graph.g2o");
    for(int i=0; i<keyframes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;

      keyframes[i]->dump(sst.str());
    }

    if(map_in_world) 
    {
      std::ofstream ofs(directory + "/map_in_world.yaml");
      YAML::Node data;
      auto matrix = map_in_world->matrix();
      std::vector<double> matrixVector(matrix.data(), matrix.data()+matrix.size());
      data["matrix"] = matrixVector;

      ofs << data ;
      ofs.close();
    }

    res.success = true;
    return true;
  }

  /**
   * @brief save map data as pcd
   * @param req
   * @param res
   * @return
   */
  bool save_map_service(hdl_graph_slam::SaveMapRequest& req, hdl_graph_slam::SaveMapResponse& res) {
    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, req.resolution);
    if(!cloud) {
      res.success = false;
      return true;
    }

    if(zero_utm && req.utm) 
    {
      for(auto& pt : cloud->points) 
        pt.getVector3fMap() += (*zero_utm).cast<float>();
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    if(zero_utm)
    {
      std::ofstream ofs(req.destination + ".utm");
      ofs << std::fixed << std::setprecision(3) << (*zero_utm).transpose() << "\t";

      if(zero_orientation)
         ofs << std::fixed << std::setprecision(7) << (*zero_orientation).coeffs().transpose() << "\r\n";
          
      ROS_INFO("[%s] Save %s ok.",__NAME__, (req.destination + ".utm").c_str());
    }

    int ret = pcl::io::savePCDFileBinary(req.destination, *cloud);
    res.success = ret == 0;

    return true;
  }
  
  bool save_odom_service(hdl_graph_slam::SaveOdom::Request& req, hdl_graph_slam::SaveOdom::Response& res) 
  {
		std::ofstream odom_file(req.destination);
		if(!odom_file.is_open())
		{
			res.info = std::string("open ") + req.destination + "failed!!";
			res.success = false;
			return true;
		}
		res.info  = std::string("File ") + req.destination + "saved successfully";
		
		odom_file << "point_cloud_registrate_odom_x_y_z\t" << "optimizated_odom_x_y_z\t" <<"gps_x_y_z"<<  std::endl;
		
		for(int i=0; i<keyframes.size(); i++) 
		{
			Eigen::Vector3d pos1 = keyframes[i]->odom.translation(); //the odom from pointcloud registration
			Eigen::Vector3d pos2 = keyframes[i]->node->estimate().translation(); //the odom after optimization
			Eigen::Vector3d pos3 = *(keyframes[i]->utm_coord);  //the utm point from gps
			odom_file << std::fixed << std::setprecision(3) << std::setw(7);
			odom_file <<pos1[0] << "\t" << pos1[1] << "\t" << pos1[2]<< "\t"
					  <<pos2[0] << "\t" << pos2[1] << "\t" << pos2[2]<< "\t"
					  <<pos3[0] << "\t" << pos3[1] << "\t" << pos3[2]<< std::endl;
					  
					  
		}
		odom_file.close();
  		return true;
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
  std::unique_ptr<message_filters::Synchronizer<MySyncPolicy1>> sync1;

  bool use_gps_instead_lidar_odom;
  ros::Subscriber gps_sub;
  ros::Subscriber nmea_sub;
  ros::Subscriber navsat_sub;
  
  ros::Subscriber utm_sub;

  ros::Subscriber imu_sub;
  ros::Subscriber floor_sub;

  ros::Publisher markers_pub;
  ros::Publisher transed_points_pub;

  std::string map_frame_id;
  std::string odom_frame_id;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix4f trans_odom2map;
  ros::Publisher odom2map_pub;

  std::string points_topic;
  ros::Publisher read_until_pub;
  ros::Publisher map_points_pub;

  tf::TransformListener tf_listener;
  tf::TransformBroadcaster odom_by_gps_broadcaster;
  
  ros::ServiceServer dump_service_server;
  ros::ServiceServer save_map_service_server;
  ros::ServiceServer save_odom_service_server;

  // keyframe queue
  std::string base_frame_id;
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;

  // gps queue
  double gps_time_offset;
  double gps_edge_stddev_xy;
  double gps_edge_stddev_z;
  boost::optional<Eigen::Isometry3d> map_in_world;
  boost::optional<Eigen::Vector3d> zero_utm;
  boost::optional<Eigen::Quaterniond> zero_orientation;
  boost::optional<Eigen::Isometry3d> gps_to_base;
  
  std::mutex gps_queue_mutex;
  std::deque<geographic_msgs::GeoPointStampedConstPtr> gps_queue;
  
  //utm queue
  double utm_time_offset;
  bool enable_utm_xy;
  double utm_edge_stddev_xy;
  double utm_edge_stddev_z;
  
  std::mutex utm_queue_mutex;
  std::deque<nav_msgs::Odometry::Ptr> utm_queue;
  

  // imu queue
  double imu_time_offset;
  bool enable_imu_orientation;
  double imu_orientation_edge_stddev;
  bool enable_imu_acceleration;
  double imu_acceleration_edge_stddev;
  std::mutex imu_queue_mutex;
  std::deque<sensor_msgs::ImuConstPtr> imu_queue;

  // floor_coeffs queue
  bool use_floor_edge;
  double floor_edge_stddev;
  std::mutex floor_coeffs_queue_mutex;
  std::deque<hdl_graph_slam::FloorCoeffsConstPtr> floor_coeffs_queue;

  // for map cloud generation
  double map_cloud_resolution;
  std::mutex keyframes_snapshot_mutex;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;

  // graph slam
  // all the below members must be accessed after locking main_thread_mutex
  std::mutex main_thread_mutex;

  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  g2o::VertexPlane* floor_plane_node;
  std::vector<KeyFrame::Ptr> keyframes;
  std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

  std::unique_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  std::unique_ptr<NmeaSentenceParser> nmea_parser;

  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
};

}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::HdlGraphSlamNodelet, nodelet::Nodelet)
