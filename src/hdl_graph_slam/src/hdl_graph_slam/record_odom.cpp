#include<iostream>
#include<fstream>
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>


using namespace std;

class Recorder
{
public:
	Recorder()
	{
		m_odom1Ok = m_odom2Ok = false;
	}
	~Recorder()
	{
		if(m_outFile.is_open())
			m_outFile.close();
	}
	
	void gps_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		m_odom1Ok = true;
		m_gpsOdom = *msg;
	}
	
	void slam_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		m_odom2Ok = true;
		m_slamOdom = *msg;
	}
	
	void timer_callback(const ros::TimerEvent&)
	{
		if(!m_odom1Ok || !m_odom2Ok )
			return ;
		static size_t count = 0;
		geometry_msgs::Transform gps_in_world;
		gps_in_world.translation.x = m_gpsOdom.pose.pose.position.x;
		gps_in_world.translation.y = m_gpsOdom.pose.pose.position.y;
		gps_in_world.translation.z = m_gpsOdom.pose.pose.position.z;
	
		gps_in_world.rotation.x =  m_gpsOdom.pose.pose.orientation.x;
		gps_in_world.rotation.y =  m_gpsOdom.pose.pose.orientation.y;
		gps_in_world.rotation.z =  m_gpsOdom.pose.pose.orientation.z;
		gps_in_world.rotation.w =  m_gpsOdom.pose.pose.orientation.w;
	
		Eigen::Isometry3d gps_in_world_isometry;
		tf::transformMsgToEigen(gps_in_world, gps_in_world_isometry);
		
		auto base_in_world_isometry = m_gps_in_base_isometry.inverse() * gps_in_world_isometry;
		auto translation = base_in_world_isometry.translation();
		
		m_outFile << fixed << setprecision(3) 
						<< translation[0] << "\t"
						<< translation[1] << "\t"
						<< m_slamOdom.pose.pose.position.x << "\t"
						<< m_slamOdom.pose.pose.position.y << std::endl;
		
		ROS_INFO("%5ld: recording...",++count);
	}
	
	bool init()
	{
		ros::NodeHandle nh;
		ros::NodeHandle nh_private("~");
		
		std::string gps_odom_topic = nh_private.param<string>("gps_odom_topic","/gps_odom");
		std::string slam_odom_topic = nh_private.param<string>("slam_odom_topic","/location/odom");
		
		m_subGpsOdom = nh.subscribe(gps_odom_topic, 1, &Recorder::gps_odom_callback,this);
		m_subSlamOdom = nh.subscribe(slam_odom_topic ,1, &Recorder::slam_odom_callback, this);
		mTimer = nh.createTimer(ros::Duration(nh_private.param<float>("record_duration",0.1)),&Recorder::timer_callback, this);
		
		string file_name = nh_private.param<string>("file_name", "");
		
		m_outFile.open(file_name);
		if(!m_outFile.is_open()) 
		{
			ROS_ERROR("[%s] open %s failed!!!",ros::this_node::getName().c_str(), file_name.c_str());
			return false;
		}
		
		ROS_INFO("open %s ok ...",file_name.c_str());
		
		tf::StampedTransform tf_gps2base;
		try
		{
			m_tf_listener.waitForTransform("base_link" ,"gps", ros::Time(0), ros::Duration(1.0));
			m_tf_listener.lookupTransform("base_link" ,"gps", ros::Time(0), tf_gps2base);
		} catch (std::exception& e) 
		{
			std::cerr << "failed to find the transform from [base_link] to [ << gps << ]!!" << std::endl;
			return false;
		}
		tf::transformTFToEigen(tf_gps2base.inverse(), m_gps_in_base_isometry);
	
		return true;
	}


private:
	ros::Subscriber m_subGpsOdom;
	ros::Subscriber m_subSlamOdom;
	
	ros::Timer mTimer;
	nav_msgs::Odometry m_gpsOdom;
	nav_msgs::Odometry m_slamOdom;
	
	ofstream m_outFile;
	
	bool m_odom1Ok, m_odom2Ok;
	tf::TransformListener m_tf_listener;
	Eigen::Isometry3d m_gps_in_base_isometry;
	
};

int main(int argc, char** argv)
{
	ros::init(argc,argv,"recor_odom_node");
	Recorder recorder;
	if(!recorder.init())
		return 0;
	ROS_INFO("%s initial ok...",ros::this_node::getName().c_str());
	ros::spin();
	return 0;
}


