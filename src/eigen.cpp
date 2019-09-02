#include <Eigen/Dense>
#include<iostream>
#include<algorithm>

using namespace std;

int main()
{
	Eigen::Vector4f coffs({0.005,0.015,0.999,1.79});
	Eigen::Vector4f point({0,0,-1.5,1});
	double val = coffs.dot(point);
	cout <<coffs.head<3>().dot(coffs.head<3>()) << endl;
	
	vector<int> array(10,1);
	vector<int> a;
	//a.reserve(10);
	a.resize(10);
	for(auto val:a)
		cout << val <<"\t";
	cout << endl;
	
	copy_if(array.begin(),array.end(),std::front_inserter(a),
		[](const int &val)
		{
			return true;
		});
		
	for(auto val:a)
		cout << val <<"\t";
	cout << endl;
}
