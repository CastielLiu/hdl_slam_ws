#include<iostream>
#include<sstream>
#include<vector>
using namespace std;


int main()
{
	std::string str("198.2 199 88\n");
	stringstream ss;
	ss << str;
	
	while(!ss.eof())
	{
		double a;
		ss >> a;
		cout << a << endl;
	}
	
	return 0;
	
}
