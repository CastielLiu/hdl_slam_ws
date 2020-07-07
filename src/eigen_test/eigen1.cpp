#include <iostream>
 
#include <Eigen/Core>
#include <Eigen/Geometry>
#define PI (3.1415926535897932346f)
 
int main(int argc, char **argv) 
{
	Eigen::Quaterniond q = Eigen::Quaterniond::Identity(); 

	std::cout << q.coeffs().transpose() << std::endl;


	std::cin.ignore();
	return 0;
}
