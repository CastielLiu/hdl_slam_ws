#include<ros/ros.h>
#include <memory>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <Eigen/Dense>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <ros/ros.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include<geometry_msgs/Transform.h>
#include <eigen_conversions/eigen_msg.h>
#include<gps_msgs/Inspvax.h>
#include<fstream>

bool first = true;
Eigen::Isometry3d map_in_world_isometry;
Eigen::Vector3d xyz_zero;

void utm_callback(const nav_msgs::Odometry::ConstPtr& utm_odom_msg)
{
	geometry_msgs::Transform pose_in_world;
	pose_in_world.translation.x = utm_odom_msg->pose.pose.position.x;
	pose_in_world.translation.y = utm_odom_msg->pose.pose.position.y;
	pose_in_world.translation.z = utm_odom_msg->pose.pose.position.z;
	
	pose_in_world.rotation.x = utm_odom_msg->pose.pose.orientation.x;
	pose_in_world.rotation.y = utm_odom_msg->pose.pose.orientation.y;
	pose_in_world.rotation.z = utm_odom_msg->pose.pose.orientation.z;
	pose_in_world.rotation.w = utm_odom_msg->pose.pose.orientation.w;
	
	Eigen::Isometry3d pose_in_world_isometry;
	tf::transformMsgToEigen(pose_in_world, pose_in_world_isometry);
	
	Eigen::Vector3d xyz = Eigen::Vector3d(pose_in_world.translation.x,pose_in_world.translation.y,pose_in_world.translation.z);
	if(first)
	{
		first = false;
		map_in_world_isometry = pose_in_world_isometry;
		xyz_zero = xyz;
	}
	
	Eigen::Vector3d transXYZ = map_in_world_isometry.inverse()*xyz;
	
	xyz -= xyz_zero;
	
	Eigen::Isometry3d odom = map_in_world_isometry.inverse() * pose_in_world_isometry;
	
	static std::ofstream outfile("/home/zwei/wendao/hdl_slam_ws/src/hdl_graph_slam/debug/data.txt");
	
	outfile << std::fixed << std::setprecision(3);
	
	outfile << odom.translation()[0] <<"\t" << odom.translation()[1] << "\t" << odom.translation()[2] <<"\t";
	outfile << xyz[0] <<"\t" << xyz[1] << "\t" << xyz[2] <<"\t";
	outfile << transXYZ[0] <<"\t" << transXYZ[1] << "\t" << transXYZ[2] <<"\n";
	
	ROS_INFO("recording ...");
	outfile.flush();
	
}

double deg2rad(const double& deg)
{
	return deg*M_PI/180.0;
}

void inspvax_callback(const gps_msgs::Inspvax::ConstPtr& inspvax_msg)
{
	nav_msgs::Odometry::Ptr ll2utm_msg(new nav_msgs::Odometry);
	ll2utm_msg->header.stamp = inspvax_msg->header.stamp;
	ll2utm_msg->header.frame_id = "world";
	
	geographic_msgs::GeoPoint point;
	point.latitude = inspvax_msg->latitude;
	point.longitude = inspvax_msg->longitude;
	point.altitude = inspvax_msg->height;
	
	geodesy::UTMPoint utm;
	geodesy::fromMsg(point, utm);
	
	ll2utm_msg->pose.pose.position.x = utm.easting;
	ll2utm_msg->pose.pose.position.y = utm.northing;
	ll2utm_msg->pose.pose.position.z = utm.altitude;
	
	Eigen::AngleAxisd rollAngle(deg2rad(inspvax_msg->roll), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(-deg2rad(inspvax_msg->azimuth), Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd pitchAngle(deg2rad(inspvax_msg->pitch), Eigen::Vector3d::UnitX());
	Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
	
	ll2utm_msg->pose.pose.orientation.x = q.x();
	ll2utm_msg->pose.pose.orientation.y = q.y();
	ll2utm_msg->pose.pose.orientation.z = q.z();
	ll2utm_msg->pose.pose.orientation.w = q.w();
	
	utm_callback(ll2utm_msg);
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"translation_test");
	ros::NodeHandle nh;
	//ros::Subscriber sub_odom = nh.subscribe("/ll2utm_odom",10,&utm_callback);
	ros::Subscriber sub_inspvax = nh.subscribe("/gps",10,&inspvax_callback);
	ros::spin();
	
	return 0;
}
