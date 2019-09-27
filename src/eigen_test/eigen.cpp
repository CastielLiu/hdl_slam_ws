#include <Eigen/Dense>
#include<iostream>
#include<algorithm>
#include<fstream>
#include <memory>

using namespace std;

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
	return q;
}

Eigen::Matrix3d euler2Matrix(const double roll, const double pitch, const double yaw)
{
	Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()));
	 
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix=yawAngle*pitchAngle*rollAngle;
	return rotation_matrix;
}

double deg2rad(const double& deg)
{
	return deg*M_PI/180.0;
}


int main()
{
//	Eigen::Vector4f coffs({0.005,0.015,0.999,1.79});
//	Eigen::Vector4f point({0,0,-1.5,1});
//	//向量内积
//	double val = coffs.dot(point);
//	//coffs.head<3>() 取向量的前三个值
//	double val2 = coffs.head<3>().dot(coffs.head<3>());
//	
//	Eigen::Quaterniond q1 = Eigen::Quaterniond(0.35, 0.2, 0.3, 0.1);
//	cout << q1.x() << "  "<< q1.y() << "  " << q1.z()  << "  "<< q1.w()  << endl;
//	q1 = q1.normalized(); //归一化
//	
//	cout << q1.x() << "  "<< q1.y() << "  " << q1.z()  << "  "<< q1.w()  << endl;
//	cout << q1.coeffs() << endl;
//	
//	
//	Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) ); 
//	Eigen::Isometry3d T=Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵

//	T.rotate ( rotation_vector.matrix() );                                     // 按照rotation_vector进行旋转

//	T.pretranslate ( Eigen::Vector3d ( 1,3,4 ) );                     // 把平移向量设成(1,3,4)

//	cout << "Transform matrix = \n" << T.matrix() <<endl;
//	
//	Eigen::Vector3d pos(1.0,2.0,3.0);
//	

//	Eigen::Vector3d eulerAngle(-(90.0-81.)/180.0*M_PI,0.0,0.0);
//	Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
//	Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
//	Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ()));
//	
//	Eigen::Matrix3d rotation_matrix;
//	rotation_matrix=yawAngle*pitchAngle*rollAngle;
//	
//	
	
	
	Eigen::Vector3d world2base_t = Eigen::Vector3d(3, 1, 0);
	Eigen::Matrix3d world2base_m = euler2Matrix(0,0.0, deg2rad(30.0));
	Eigen::Isometry3d world2baseT = Eigen::Isometry3d::Identity();
	world2baseT.rotate(world2base_m);
	world2baseT.pretranslate(world2base_t);
	
	Eigen::Vector3d world2map_t = Eigen::Vector3d(3, 0, 0);
	Eigen::Matrix3d world2map_m = euler2Matrix(0,0,deg2rad(45.0));
	Eigen::Isometry3d world2mapT = Eigen::Isometry3d::Identity();
	world2mapT.rotate(world2map_m);
	world2mapT.pretranslate(world2map_t);
	
	Eigen::Vector3d xyz(2,1,0);
	cout << world2mapT.inverse() * Eigen::Vector3d(3,3,0) << endl;
	cout << world2mapT.inverse() * Eigen::Vector3d(0,3,0) << endl;
	cout << world2mapT.inverse() * Eigen::Vector3d(1,2,0) << endl;
	
	return 0;
	
	cout << world2base_m << endl;
	cout << world2map_m << endl;
	
	Eigen::Isometry3d map2baseT = world2mapT.inverse() * world2baseT;
	cout << map2baseT.linear().eulerAngles(2,1,0)*180.0/M_PI << endl;
	auto t = map2baseT.translation();
	cout << map2baseT.translation() << endl;
	cout << sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2] ) << endl;
	cout << " .............." << endl;

	
	
	cout << world2baseT.inverse()*xyz << endl;
	cout << world2mapT.inverse()*xyz << endl;
	
	cout << map2baseT.inverse() * xyz << endl;
	
	cout << map2baseT.inverse() * world2mapT.inverse() * world2baseT * xyz << endl;
	
	
	
//copy 与 copy_if 只负责拷贝，不负责开辟内存
//	copy_if(array.begin(),array.end(),std::back_inserter(a),
//		[](const int &val)
//		{
//			return true;
//		});
		
 
}
