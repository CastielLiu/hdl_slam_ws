#include <Eigen/Dense>
#include<iostream>
#include<algorithm>
#include<fstream>
#include <memory>

using namespace std;

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

	Eigen::Vector3d eulerAngle(-(90.0-81.)/180.0*M_PI,0.0,0.0);
	Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ()));
	
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix=yawAngle*pitchAngle*rollAngle;
	
	//cout << rotation_matrix << endl;
	
	
	ifstream in_file("add_gps.txt");
	ofstream out_file("new.txt");
	
	if(!in_file)
		return 0;
	
	string str;
	getline(in_file,str);
	
	cout << str << endl;
	
	float nothing[6], x,y,z;
	
	while(in_file >> nothing[0] >> nothing[1] >>nothing[2] >> nothing[3]>>nothing[4]>>nothing[5] >> x >> y >>z )
	{
		Eigen::Vector3d Point(x,y,z);
		Eigen::Vector3d newPoint = rotation_matrix * Point;
		for(int i=0; i<6; ++i)
			out_file << nothing[i] << "\t";
		out_file << newPoint[0] << "\t" <<newPoint[1] << "\t" <<newPoint[2] << "\r\n";
			
		
		cout << x <<" " << y <<" " << z << endl;
	}
	
	
	 std::unique_ptr<Eigen::Isometry3d> updateOdomMatrix_; 
	 
	if(updateOdomMatrix_)
		cout << "sdfsdfsda" << endl;
	
	
	
//copy 与 copy_if 只负责拷贝，不负责开辟内存
//	copy_if(array.begin(),array.end(),std::back_inserter(a),
//		[](const int &val)
//		{
//			return true;
//		});
		
 
}
