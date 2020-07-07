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
		
		std::string out_file_prefix = nh_private.param<std::string>("out_file_prefix","");
		if(out_file_prefix.empty())
		{
			ROS_ERROR("[%s] please set out_file_prefix int launch file!", __NAME__);
			return false;
		}
		out_file_prefix = out_file_prefix.substr(0,out_file_prefix.find_first_of('.'));
		
		std::string gps_odom_file = out_file_prefix + "_gps_odom.txt";
		std::string slam_odom_file = out_file_prefix + "_slam_odom.txt";
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
	
	inline double deg2rad(double deg)
	{
		return deg/180.0*M_PI;
	}
	
	inline double rad2deg(double rad)
	{
		return rad*180.0/M_PI;
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
		double x,y,z, quat_x, quat_y, quat_z, quat_w;
		std::stringstream ss(line);
		ss >> x >> y >> z >> quat_x >> quat_y >>  quat_z >> quat_w;

		in_file.close();
		 
		Eigen::Matrix3d rotation_matrix = Eigen::Quaterniond(quat_w, quat_x, quat_y, quat_z).normalized().matrix();
		
		map_in_world_ = Eigen::Isometry3d::Identity();
		map_in_world_.rotate(rotation_matrix); 
		map_in_world_.pretranslate(Eigen::Vector3d(x,y,0)); 

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
		
			tf::StampedTransform tf_gps_in_base;
			try
			{
				tf_listener_.waitForTransform("base_link" ,gps_frame_id, ros::Time(0), ros::Duration(1.0));
				tf_listener_.lookupTransform("base_link" , gps_frame_id, ros::Time(0), tf_gps_in_base);
			} 
			catch (std::exception& e) 
			{
				std::cerr << "failed to find the transform from [base_link] to [" << gps_frame_id << "]!!" << std::endl;
				return Eigen::Isometry3d::Identity();
			}

			tf::transformTFToEigen(tf_gps_in_base.inverse(), gps_to_base);

			parsed = true;
		}
		//由gps定位点与gps在base_link的安装位置,求base_link的大地坐标
		Eigen::Isometry3d gps_in_world = odom2isometry(odom);
		Eigen::Isometry3d base_in_world = gps_in_world * gps_to_base;

		return map_in_world_.inverse() * base_in_world; //base_in_map
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
		
		float speed = odom_msg->twist.twist.linear.y;
		
		Eigen::Vector3d euler = matrix2Euler(matrix);
		float yaw_angle = euler[2] ;
		if(yaw_angle < 0) yaw_angle+= M_PI * 2; //将-180->180 转为0-360.0
		
		of_gps_odom_ << std::fixed << std::setprecision(3)
					 << translation(0) << "\t" << translation(1) << "\t" 
					 << yaw_angle*180.0/M_PI << "\t" << speed << "\r\n";
					 
		of_gps_odom_.flush();
	}
	
	void slam_odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
		const geometry_msgs::Quaternion& ori_msg = odom_msg->pose.pose.orientation;
		Eigen::Quaterniond ori(ori_msg.w, ori_msg.x, ori_msg.y, ori_msg.z);
		ori.normalize();
		
		Eigen::Vector3d euler = quaternion2Euler(ori);
		float yaw_angle = euler[2];
		
		if(yaw_angle < 0) yaw_angle+= M_PI * 2; //将-180->180 转为0-360.0
		float x_speed = odom_msg->twist.twist.linear.x;
		float y_speed = odom_msg->twist.twist.linear.y;
		//float speed = sqrt(x_speed*x_speed + y_speed*y_speed);
		float speed = x_speed*cos(yaw_angle) + y_speed*sin(yaw_angle);
		
		of_slam_odom_ << std::fixed << std::setprecision(3)
					  << odom_msg->pose.pose.position.x << "\t" << odom_msg->pose.pose.position.y << "\t" 
					  << yaw_angle*180.0/M_PI << "\t" << speed << "\r\n";
//		std::cout << std::fixed << std::setprecision(2);
//		std::cout << odom_msg->twist.twist.linear.x << "\t" << odom_msg->twist.twist.linear.y << "\t" << odom_msg->twist.twist.linear.z<< std::endl;
		of_slam_odom_.flush();
	}
	
	Eigen::Quaterniond euler2Quaternion(double x_angle, double y_angle, double z_angle)
	{
		Eigen::AngleAxisd xAngle(x_angle, Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd yAngle(y_angle, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd zAngle(z_angle, Eigen::Vector3d::UnitZ());
		
		Eigen::Quaterniond q = zAngle * yAngle * xAngle;

		return q.normalized();
	}
	
	Eigen::Vector3d quaternion2Euler(const Eigen::Quaterniond& quat)
	{
		Eigen::Vector3d angles = quat.matrix().eulerAngles(2,1,0);
		double x_angle = angles[2];
		double y_angle = angles[1];
		double z_angle = angles[0];
		
		if(y_angle < -M_PI/2 || y_angle > M_PI/2)
		{
			x_angle += M_PI;
			if(x_angle < -M_PI)      x_angle += 2*M_PI;
			else if(x_angle > M_PI)  x_angle -= 2*M_PI;
		
			y_angle = M_PI - y_angle;
			if(y_angle > M_PI) y_angle -= 2*M_PI;
			
			z_angle -= M_PI;
		}
		
		return Eigen::Vector3d(x_angle,y_angle,z_angle);
	}
	
	Eigen::Vector3d matrix2Euler(const Eigen::Matrix3d& matrix)
	{
		Eigen::Vector3d angles = matrix.eulerAngles(2,1,0);
		double x_angle = angles[2];
		double y_angle = angles[1];
		double z_angle = angles[0];
		
		if(y_angle < -M_PI/2 || y_angle > M_PI/2)
		{
			x_angle += M_PI;
			if(x_angle < -M_PI)      x_angle += 2*M_PI;
			else if(x_angle > M_PI)  x_angle -= 2*M_PI;
		
			y_angle = M_PI - y_angle;
			if(y_angle > M_PI) y_angle -= 2*M_PI;
			
			z_angle -= M_PI;
		}
		
		return Eigen::Vector3d(x_angle,y_angle,z_angle);
	}
	
	
	void broadcastTf(float roll,float pitch, float yaw)
	{
		Eigen::Quaterniond quat = euler2Quaternion(deg2rad(roll),deg2rad(pitch),deg2rad(yaw));
	
		Eigen::Vector3d angles = quaternion2Euler(quat);
		float x_angle = rad2deg(angles[0]);
		float y_angle = rad2deg(angles[1]);
		float z_angle = rad2deg(angles[2]);

		std::cout << std::fixed << std::setprecision(2);

		if(int(roll)!=int(x_angle) || int(pitch)!=int(y_angle) || int(yaw)!=int(z_angle))
		{
			std::cout << roll  << "\t" << pitch << "\t" << yaw << std::endl;
			std::cout << x_angle  << "\t" << y_angle << "\t" << z_angle << "\t" << std::endl;
		}
		else
		{
			std::cout << "." ;
		}
		

		geometry_msgs::Quaternion odom_quat;
		odom_quat.w = quat.w();
		odom_quat.x = quat.x();
		odom_quat.y = quat.y();
		odom_quat.z = quat.z();

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = "world";
		odom_trans.child_frame_id = "gps";

		odom_trans.transform.translation.x = 0;
		odom_trans.transform.translation.y = 0;
		odom_trans.transform.translation.z = 0;
		odom_trans.transform.rotation = odom_quat;
		tf_broadcaster_.sendTransform(odom_trans);
	}
	
	void testQuaternoin()
	{
		ROS_INFO("testQuaternoin start.");
		double start1 = -45.0 , end1 = 45.0;
		double increment = 5;
		for(double x_angle=start1; x_angle<=end1 && ros::ok(); x_angle+=increment)
		{
			double start2 = -45.0 , end2 = 45.0;
			for(double y_angle=start2; y_angle<=end2 && ros::ok() ; y_angle+=increment)
			{
				double start3 = -180.0, end3 = 180;
				for(double z_angle=start3; z_angle<=end3 && ros::ok(); z_angle+=increment)
				{
					broadcastTf(x_angle,y_angle,z_angle);
				}
			}
		}
		
		ROS_INFO("testQuaternoin stop.");
	}

private:
	ros::Subscriber sub_slam_odom_;
	ros::Subscriber sub_gps_odom_;
	std::ofstream of_gps_odom_;
	std::ofstream of_slam_odom_;
	Eigen::Isometry3d map_in_world_;
	tf::TransformListener tf_listener_;
	tf::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char **argv)
{
	ros::init(argc,argv, "record_odom_node");
	OdomRecorder recorder;
	if(recorder.init())
	{
		//recorder.testQuaternoin();
		ros::spin();
	}
		
	
	return true;
}
