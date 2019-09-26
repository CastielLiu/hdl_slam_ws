#include <iostream>
 
#include <Eigen/Core>
#include <Eigen/Geometry>
#define PI (3.1415926535897932346f)
 
int main(int argc, char **argv) 
{
    using ::std::cout;
    using ::std::endl;
    double yaw = PI/3,pitching = 0,droll = 0;
    
    
      //EulerAngles to RotationMatrix
    ::Eigen::Vector3d ea0(yaw,pitching,droll);
    ::Eigen::Matrix3d R;
    R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
        * ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
        * ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
 
    cout << R << endl << endl;
 
    //RotationMatrix to Quaterniond
    ::Eigen::Quaterniond q; 
    q = R;    
    cout << q.x() << endl << endl;
    cout << q.y() << endl << endl;
    cout << q.z() << endl << endl;
    cout << q.w() << endl << endl;
    
    //Quaterniond to RotationMatrix
    ::Eigen::Matrix3d Rx = q.toRotationMatrix();
    cout << Rx << endl << endl;
    
    
    //RotationMatrix to EulerAngles
    ::Eigen::Vector3d ea1 = Rx.eulerAngles(2,1,0);     
    cout << ea1/PI*180 << endl << endl;
    
    
    std::cin.ignore();
    return 0;
}
