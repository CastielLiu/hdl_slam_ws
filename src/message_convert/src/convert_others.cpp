#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
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

using std::string;


class Convert
{
public:
	Convert(){}
	~Convert(){}
	bool init()
	{
		ros::NodeHandle nh;
		ros::NodeHandle nh_("~");
		
		bool is_utm = nh_.param<bool>("is_sub_utm","false");
		
		if(is_utm)
			sub_utm_ = nh.subscribe(nh_.param<string>("in_topic_utm","/in_utm"),20,&Convert::utm_callback,this);
		else
			sub_inspvax_ = nh.subscribe("/gps",10,&Convert::inspvax_callback,this);
		pub_utm_ = nh.advertise<nav_msgs::Odometry>(nh_.param<string>("out_topic_utm","/out_utm"),20);
	}
	void utm_callback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		auto newMsg = *msg;
		newMsg.header.stamp = ros::Time::now();
		newMsg.header.frame_id = "world";
		pub_utm_.publish(newMsg);
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
		ll2utm_msg->child_frame_id = "gps";
	
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

//		Eigen::AngleAxisd rollAngle(deg2rad(0.0), Eigen::Vector3d::UnitY());
//		Eigen::AngleAxisd yawAngle(-deg2rad(inspvax_msg->azimuth), Eigen::Vector3d::UnitZ());
//		Eigen::AngleAxisd pitchAngle(deg2rad(0.0), Eigen::Vector3d::UnitX());

		Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
	
		ll2utm_msg->pose.pose.orientation.x = q.x();
		ll2utm_msg->pose.pose.orientation.y = q.y();
		ll2utm_msg->pose.pose.orientation.z = q.z();
		ll2utm_msg->pose.pose.orientation.w = q.w();
	
		utm_callback(ll2utm_msg);
	}

private:
	string gps_frame_id_;
	ros::Subscriber sub_utm_;
	ros::Subscriber sub_inspvax_;
	ros::Publisher pub_utm_;
};

int main(int argc, char** argv)
{
	ros::init(argc,argv,"convert_others_node");
	Convert convert;
	if(convert.init())
		ros::spin();
	return 0;
}

