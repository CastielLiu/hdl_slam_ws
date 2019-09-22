#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
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
		nh_.param<std::string>("gps_frame_id",gps_frame_id_,"/gps_odom");
		sub_utm_ = nh.subscribe(nh_.param<string>("in_topic_utm","/in_utm"),20,&Convert::utm_callback,this);
		pub_utm_ = nh.advertise<nav_msgs::Odometry>(nh_.param<string>("out_topic_utm","/out_utm"),20);
	}
	void utm_callback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		auto newMsg = *msg;
		newMsg.header.stamp = ros::Time::now();
		newMsg.header.frame_id = gps_frame_id_;
		pub_utm_.publish(newMsg);
	}
private:
	string gps_frame_id_;
	ros::Subscriber sub_utm_;
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

