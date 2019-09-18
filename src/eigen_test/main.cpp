#include<Eigen/Dense>
#include<iostream>
#include<deque>

using namespace std;

int main()
{
	Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Identity();
	information_matrix.block<2, 2>(1, 1) /= 5;
	
	cout << information_matrix << endl;
	
	std::deque<int> num_deque(10);
	int i=0;
	for(auto& num:num_deque)
		num = i++;
	
	for(auto& num:num_deque)
		cout << hex << &num  <<": "<<  dec << num<< endl;
	cout << endl;	
	
	num_deque.erase(num_deque.begin(),num_deque.end()-2);
	num_deque.push_back(11);
	
	for(auto& num:num_deque)
		cout << hex << &num  <<": "<<  dec << num<< endl;
	
	int a=10, b=20;
	const int *ptr = &a;
	ptr = &b;
	
	
	return 0;
}
