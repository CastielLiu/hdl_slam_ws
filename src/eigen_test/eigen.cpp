#include <Eigen/Dense>
#include <iostream>
#include <iomanip>

using namespace std;


inline double rad2deg(double rad)
{
	return rad*180.0/M_PI;
}

inline double deg2rad(double deg)
{
	return deg/180.0*M_PI;
}

Eigen::Quaterniond euler2Quaternion(double x_theta, double y_theta, double z_theta)
{
	Eigen::AngleAxisd xAngle(x_theta, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yAngle(y_theta, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd zAngle(z_theta, Eigen::Vector3d::UnitZ());
	Eigen::Quaterniond q = zAngle * yAngle * xAngle;
	return q.normalized();
}

Eigen::Vector3d quaternion2Euler(const Eigen::Quaterniond& quat)
{
    Eigen::Vector3d angles = quat.matrix().eulerAngles(2,1,0);
    double x_angle = angles[2];
    double y_angle = angles[1];
    double z_angle = angles[0];
    
    if(y_angle<-M_PI/2 || y_angle>M_PI/2)
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

void testQuaternoin()
{
	std::cout << "testQuaternoin start." << std::endl;
	double start1 = -45.0 , end1 = 45.0;
	double increment = 5;
	for(double x_angle=start1; x_angle<=end1 ; x_angle+=increment)
	{
		double start2 = -45.0 , end2 = 45.0;
		for(double y_angle=start2; y_angle<=end2  ; y_angle+=increment)
		{
			double start3 = -180.0, end3 = 180;
			for(double z_angle=start3; z_angle<=end3 ; z_angle+=increment)
			{
				Eigen::Quaterniond quat = euler2Quaternion(deg2rad(x_angle), deg2rad(y_angle), deg2rad(z_angle));
				Eigen::Vector3d angles = quaternion2Euler(quat);
				float x_angle_2 = rad2deg(angles[0]);
				float y_angle_2 = rad2deg(angles[1]);
				float z_angle_2 = rad2deg(angles[2]);
				
				std::cout << std::fixed << std::setprecision(2);

				//此处比较的时候进行了取整处理，因为浮点数精度的问题，转换前后可能会有一定偏差
				if(int(x_angle_2)!=int(x_angle) || int(y_angle_2)!=int(y_angle) || int(z_angle_2)!=int(z_angle))
				{
					std::cout << x_angle_2  << "\t" << y_angle_2 << "\t" << z_angle_2 << std::endl;
					std::cout << x_angle  << "\t" << y_angle << "\t" << z_angle << "\t" << std::endl;
				}
			}
		}
	}

	std::cout << "testQuaternoin stop." << std::endl;
}



int main()
{
	testQuaternoin();
	return 0;
}
