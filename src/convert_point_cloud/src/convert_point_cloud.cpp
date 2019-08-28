#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Convert
{
public:
	Convert(){}
	~Convert(){}
	bool init()
	{
		ros::NodeHandle nh;
		ros::NodeHandle nh_private("~");
		
		nh_private.param<std::string>("in_topic_name",in_topic_name_,"");
		nh_private.param<std::string>("out_topic_name",out_topic_name_,"");
		nh_private.param<std::string>("frame_id",frame_id_,"");
		if(in_topic_name_.empty()|| out_topic_name_.empty())
		{
			ROS_ERROR("topic name is empty!!");
			return false;
		}
		sub_point_cloud_ = nh.subscribe(in_topic_name_,10,&Convert::point_cloud_callback,this);
		pub_point_cloud_ = nh.advertise<sensor_msgs::PointCloud2>(out_topic_name_,10);
	}
	void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		point_cloud_ = *msg;
		point_cloud_.header.stamp = ros::Time::now();
		if(!frame_id_.empty())
			point_cloud_.header.frame_id = frame_id_;
		pub_point_cloud_.publish(point_cloud_);
	}
private:
	std::string in_topic_name_;
	std::string out_topic_name_;
	std::string frame_id_;
	sensor_msgs::PointCloud2 point_cloud_;
	ros::Subscriber sub_point_cloud_;
	ros::Publisher pub_point_cloud_;
};

int main(int argc, char** argv)
{
	ros::init(argc,argv,"point_cloud_convert");
	Convert convert;
	if(convert.init())
		ros::spin();
	return 0;
}

