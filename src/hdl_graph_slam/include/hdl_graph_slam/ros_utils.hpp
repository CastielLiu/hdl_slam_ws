#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

namespace hdl_graph_slam {

/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  odom_trans.header.frame_id = frame_id;
  odom_trans.child_frame_id = child_frame_id;

  odom_trans.transform.translation.x = pose(0, 3);
  odom_trans.transform.translation.y = pose(1, 3);
  odom_trans.transform.translation.z = pose(2, 3);
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}

static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, Eigen::Isometry3d& pose, const std::string& frame_id, const std::string& child_frame_id) 
{
  Eigen::Quaterniond quat(pose.linear());
  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();
  //std::cout << quat.x() << "\t" << quat.y() << "\t" << quat.z() << "\t" << quat.w() << std::endl;


  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  odom_trans.header.frame_id = frame_id;
  odom_trans.child_frame_id = child_frame_id;

  odom_trans.transform.translation.x = pose(0, 3);
  odom_trans.transform.translation.y = pose(1, 3);
  odom_trans.transform.translation.z = pose(2, 3);
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}

static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr& odom_msg) 
{
  const auto& orientation = odom_msg->pose.pose.orientation;
  const auto& position = odom_msg->pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;
//欧氏变换矩阵 // 虽然称为3d，实质上是4＊4的矩阵
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = quat.toRotationMatrix(); //四元数转旋转矩阵
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  
  return isometry;
}

static Eigen::Isometry3d odom2isometry(const nav_msgs::Odometry& odom) 
{
  const auto& orientation = odom.pose.pose.orientation;
  const auto& position = odom.pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;
//欧氏变换矩阵 // 虽然称为3d，实质上是4＊4的矩阵
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = quat.toRotationMatrix(); //四元数转旋转矩阵
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return isometry;
}


static Eigen::Matrix4f odom2matrix(const nav_msgs::OdometryConstPtr& odom_msg)
{
	const auto& orientation = odom_msg->pose.pose.orientation;
	const auto& position = odom_msg->pose.pose.position;
	Eigen::Quaternionf quat;
	quat.w() = orientation.w;
	quat.x() = orientation.x;
	quat.y() = orientation.y;
	quat.z() = orientation.z;
	
	Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
	matrix.block<3,3>(0,0) = quat.matrix();
	matrix(0,3) = position.x;
	matrix(1,3) = position.y;
	matrix(2,3) = position.z;
	
	return matrix;
}

static Eigen::Matrix4f tfTransform2matrix(const tf::Transform& trans)
{
	const auto& tf_quat = trans.getRotation();
	const auto& tf_vector3 = trans.getOrigin();
	
	Eigen::Quaternionf quat;
	quat.w() = tf_quat.getW();
	quat.x() = tf_quat.getX();
	quat.y() = tf_quat.getY();
	quat.z() = tf_quat.getZ();
	
	Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
	matrix.block<3,3>(0,0) = quat.matrix();
	matrix(0,3) = tf_vector3.getX();
	matrix(1,3) = tf_vector3.getY();
	matrix(2,3) = tf_vector3.getZ();
	
	return matrix;
}

}

#endif // ROS_UTILS_HPP
