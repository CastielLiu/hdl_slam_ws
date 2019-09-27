



//  欧拉角转四元数
Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
	Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
	return q;
}

//四元数转欧拉角 roll pitch yaw  zxy
Eigen::Vector3d eulerAngle=quaternion.matrix().eulerAngles(2,1,0);

//四元数转旋转矩阵
Eigen::Matrix3d rotation_matrix = quaternion.matrix();

//旋转矩阵转欧拉角 roll pitch yaw
Eigen::Vector3d eulerAngle = rotation_matrix.eulerAngles(2,1,0);

//欧拉角转旋转矩阵  弧度制
Eigen::Matrix3d euler2Matrix(const double roll, const double pitch, const double yaw)
{
	Eigen::AngleAxisd rollAngle(AngleAxisd(roll,Vector3d::UnitY()));
	Eigen::AngleAxisd pitchAngle(AngleAxisd(pitch,Vector3d::UnitX()));
	Eigen::AngleAxisd yawAngle(AngleAxisd(yaw,Vector3d::UnitZ()));
	 
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix=yawAngle*pitchAngle*rollAngle;
	return rotation_matrix;
}

//监听坐标变换
tf::TransformStamped transform;
tf_listener.waitForTransform(parent_frame_id, child_frame_id, ros::Time(0), ros::Duration(1.0));
tf_listener.lookupTransform(parent_frame_id , child_frame_id, ros::Time(0), transform);

//tf::Transform 转Eigen变换矩阵
void tf::transformTFToEigen (const tf::Transform &t, Eigen::Isometry3d &e);


//需要 tf_conversions 功能包
//将TransformStamped 强制类型转换为Transform
tf::transformTFToEigen(*(tf::Transform *)&transform, isometry);

//tf::TransformStamped 转 tf::Transform 
1. 利用强制类型转换，将子类转换为基类  *(tf::Transform *)&transform
2. 利用tf::TransformStamped::inverse() 返回逆矩阵，再次求逆得到tranform




//变换矩阵（4X4）
//Eigen::Isometry3d


