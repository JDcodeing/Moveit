#include <vector>
#include "spline.h"
#include <iostream>

int main()
{
	std::vector<std::vector<double> > jointvalues;
	std::vector<double> begin(6,0.0);
	jointvalues.push_back(begin);

	std::vector<double> point1(6,0.0);
	point1[0] = -0.25; 
	point1[1] = -0.94;
	point1[2] = 1.3;
	point1[3] = -9.60817e-17;
	point1[4] = 0.7;
	point1[5] = 1.7344e-16;
	jointvalues.push_back(point1);

	point1[0] = -0.4; 
	point1[1] = -0.73; 
	point1[2] = 1.7;
	point1[3] = -0.02;
	point1[4] = 0.8;
	point1[5] = 1.7344e-16;
	jointvalues.push_back(point1);

	point1[0] = -0.26; 
	point1[1] = 0.2; 
	point1[2] = 1.88;
	point1[3] = -0.03;
	point1[4] = 1.057;
	point1[5] = -0.034;
	jointvalues.push_back(point1);

	for (int i = 0; i < 6; ++i)
	{

			std::vector<double> X,Y;
		for (int j = 0; j < jointvalues.size(); ++j)
		{
			X.push_back(j);
			Y.push_back(jointvalues[j][i]);

		}
		tk::spline s;
		s.set_points(X,Y);
		std::cout << "dimension:"<<i<<"	********************!!!!!****"<<std::endl;
		for (int k = 0; k < 10; k++)
		{
			double val = 0.1*k;
			std::cout << val << " " << s(val);
		}
		std::cout << std::endl;
	
	}
	return 0;


}
