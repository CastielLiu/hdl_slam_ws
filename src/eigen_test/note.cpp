



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

//欧拉角转旋转矩阵
Eigen::Matrix3d euler2Matrix(const double roll, const double pitch, const double yaw)
{
	Eigen::AngleAxisd rollAngle(AngleAxisd(roll,Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle(AngleAxisd(pitch,Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle(AngleAxisd(yaw,Vector3d::UnitZ()));
	 
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix=yawAngle*pitchAngle*rollAngle;
	return rotation_matrix;
}


//变换矩阵（4X4）
//Eigen::Isometry3d


