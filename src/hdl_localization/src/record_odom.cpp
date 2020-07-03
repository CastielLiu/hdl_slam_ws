#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <fstream>


#define __NAME__ "record_odom_node"

class OdomRecorder
{
public:
	OdomRecorder(){}
	~OdomRecorder()
	{
		if(of_slam_odom_.is_open())
			of_slam_odom_.close();
		if(of_gps_odom_.is_open())
			of_gps_odom_.close();
	}
	bool init()
	{
		ros::NodeHandle nh, nh_private("~");
		std::string gps_odom_topic = nh_private.param<std::string>("gps_odom_topic","/gps_odom");
		sub_gps_odom_ = nh.subscribe(gps_odom_topic,10,&OdomRecorder::gps_odom_callback, this);
		
		std::string slam_odom_topic = nh_private.param<std::string>("slam_odom_topic","/slam_odom");
		sub_slam_odom_ = nh.subscribe(slam_odom_topic,10,&OdomRecorder::slam_odom_callback, this);
		
		std::string out_file_dir = nh_private.param<std::string>("out_file_dir","");
		if(out_file_dir.empty())
		{
			ROS_ERROR("[%s] please set out_file_dir int launch file!", __NAME__);
			return false;
		}
		std::string gps_odom_file = out_file_dir + "/gps_odom.txt";
		std::string slam_odom_file = out_file_dir + "/slam_odom.txt";
		of_gps_odom_.open(gps_odom_file);
		if(!of_gps_odom_.is_open())
		{
			ROS_ERROR("[%s] Open %s failed", __NAME__, gps_odom_file.c_str());
			return false;
		}
		of_slam_odom_.open(slam_odom_file);
		if(!of_slam_odom_.is_open())
		{
			ROS_ERROR("[%s] Open %s failed", __NAME__, slam_odom_file.c_str());
			return false;
		}
		
		return load_map_in_world();
	}
	
	inline double deg2rad(double & deg)
	{
		return deg/180.0*M_PI;
	}
	
	bool load_map_in_world()
	{
		ros::NodeHandle nh_private("~");
		std::string map_in_world_file = nh_private.param<std::string>("map_in_world_file","");
		if(map_in_world_file.empty())
		{
			ROS_ERROR("[%s] please set map_in_world_file int launch file!", __NAME__);
			return false;
		}
		std::ifstream in_file(map_in_world_file);
		if(!in_file.is_open())
		{
			ROS_ERROR("[%s] Open %s failed", __NAME__, map_in_world_file.c_str());
			return false;
		}
		
		std::string line;
		getline(in_file,line);
		if(line.length() == 0)
		{
			ROS_ERROR("[%s] Read %s error", __NAME__, map_in_world_file.c_str());
			return false;
		}
		double x,y,z, roll,pitch,yaw;
		std::stringstream ss(line);
		ss >> x >> y >> z >> roll >> pitch >>  yaw;
		std::cout << std::fixed << std::setprecision(3) 
			<< x << "\t" << y << "\t" << z << "\t" << roll << "\t" << pitch << "\t" <<  yaw << std::endl;
		in_file.close();
		Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(deg2rad(roll),Eigen::Vector3d::UnitY()));
		Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(deg2rad(pitch),Eigen::Vector3d::UnitX()));
		Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(deg2rad(yaw),Eigen::Vector3d::UnitZ()));
		 
		Eigen::Matrix3d rotation_matrix;
		rotation_matrix=yawAngle*pitchAngle*rollAngle;
		
		std::cout << std::fixed << std::setprecision(3) << rotation_matrix << std::endl;
		
		map_in_world_ = Eigen::Isometry3d::Identity();
		map_in_world_.rotate(rotation_matrix); 
		map_in_world_.pretranslate(Eigen::Vector3d(x,y,0)); 
		std::cout << std::fixed << std::setprecision(3) << map_in_world_.linear() << std::endl;
		std::cout << std::fixed << std::setprecision(3) << map_in_world_.translation() << std::endl;
		return true;
	}
	
	Eigen::Isometry3d get_odom_by_gps(const nav_msgs::Odometry::ConstPtr& utm_odom_msg)
	{
		nav_msgs::Odometry odom = *utm_odom_msg;
		odom.pose.pose.position.z = 0;
		
		static bool parsed = false;
		static Eigen::Isometry3d gps_to_base;

		if(!parsed)
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
				tf_listener_.waitForTransform("base_link" ,gps_frame_id, ros::Time(0), ros::Duration(1.0));
				tf_listener_.lookupTransform("base_link" , gps_frame_id, ros::Time(0), tf_gps2base);
			} 
			catch (std::exception& e) 
			{
				std::cerr << "failed to find the transform from [base_link] to [" << gps_frame_id << "]!!" << std::endl;
				return Eigen::Isometry3d::Identity();
			}

			tf::transformTFToEigen(tf_gps2base, gps_to_base);
			
			std::cout << "gps_to_base: " << std::endl;
			std::cout << std::fixed << std::setprecision(3) << gps_to_base.translation() << std::endl;
			std::cout << std::fixed << std::setprecision(3) << gps_to_base.linear() << std::endl;
			std::cout << std::fixed << std::setprecision(3) << gps_to_base.linear().eulerAngles(2,1,0).transpose() << std::endl;

			parsed = true;
		}
		//由gps定位点与gps在base_link的安装位置,求base_link的大地坐标
		Eigen::Isometry3d gps_in_world = odom2isometry(odom);
		Eigen::Isometry3d base_in_world = gps_in_world * gps_to_base;
		
		std::cout << "gps_in_world: " << std::endl;
		std::cout << std::fixed << std::setprecision(3) << gps_in_world.translation() << std::endl;
		
		std::cout << "base_in_world: " << std::endl;
		std::cout << std::fixed << std::setprecision(3) << base_in_world.translation() << std::endl;

		return map_in_world_.inverse() * base_in_world;
	}
	
	Eigen::Isometry3d odom2isometry(const nav_msgs::Odometry& odom) 
	{
	  const auto& orientation = odom.pose.pose.orientation;
	  const auto& position = odom.pose.pose.position;

	  Eigen::Quaterniond quat;
	  quat.w() = orientation.w;
	  quat.x() = orientation.x;
	  quat.y() = orientation.y;
	  quat.z() = orientation.z;

	  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
	  isometry.linear() = quat.toRotationMatrix();
	  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
	  return isometry;
	}

	void gps_odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
		Eigen::Isometry3d odom = get_odom_by_gps(odom_msg);
		
		Eigen::Vector3d translation = odom.translation();
		Eigen::Matrix3d matrix      = odom.linear();
		of_gps_odom_ << std::fixed << std::setprecision(3)
					 << translation(0) << "\t" << translation(1) << "\t" 
					 << matrix.eulerAngles(2,1,0)[0] << "\t" << odom_msg->twist.twist.linear.x << "\r\n";
		of_gps_odom_.flush();
	}
	
	void slam_odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
		const geometry_msgs::Quaternion& ori_msg = odom_msg->pose.pose.orientation;
		Eigen::Quaterniond ori(ori_msg.w, ori_msg.x, ori_msg.y, ori_msg.z);
		ori.normalize();
		
		of_slam_odom_ << std::fixed << std::setprecision(3)
					  << odom_msg->pose.pose.position.x << "\t" << odom_msg->pose.pose.position.y << "\t" 
					  << ori.matrix().eulerAngles(2,1,0)[0] << "\t" << odom_msg->twist.twist.linear.x  << "\r\n";
		of_slam_odom_.flush();
	}

private:
	ros::Subscriber sub_slam_odom_;
	ros::Subscriber sub_gps_odom_;
	std::ofstream of_gps_odom_;
	std::ofstream of_slam_odom_;
	Eigen::Isometry3d map_in_world_;
	tf::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
	ros::init(argc,argv, "record_odom_node");
	OdomRecorder recorder;
	if(recorder.init())
		ros::spin();
	
	return true;
}
