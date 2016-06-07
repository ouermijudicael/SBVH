#include <vector>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;

void createABCtracer()
{
    int num_of_tracers = 1000;


    // openfile
    ofstream myfile("ABCtracer");

    //title
    myfile << "#particle \t";
    myfile << "numPointsIntracer\t";
    myfile << "xi\t";
    myfile << "yi\t";
    myfile << "zi\t";
    myfile << "ti\t";
    myfile << "...\t";
    myfile << "xf\t";
    myfile << "yf\t";
    myfile << "zf\t";
    myfile << "tf\n";
    for(int idx_tracers=0; idx_tracers < num_of_tracers; idx_tracers++)
    {
        int num_of_steps =abs( (int)rand()/(int)(RAND_MAX/100) );
	while(num_of_steps < 5)
    	    num_of_steps = abs( (int)rand()/(int)(RAND_MAX/100) );

	double A = sqrt(3);
	double B = sqrt(2);
	double C = 1;

	double x0 = (double)rand()/(double)(RAND_MAX/1.00);
	double y0 = (double)rand()/(double)(RAND_MAX/1.00);
	double z0 = (double)rand()/(double)(RAND_MAX/1.00); 
	double t0 = (double)rand()/(double)(RAND_MAX/1.00);
	double x1, y1, z1, t1;



	// particle id
	myfile << idx_tracers <<"\t";

	// number of points
	myfile << num_of_steps <<"\t";
	// the first one in the file
	myfile << x0 << "\t";
	myfile << y0 << "\t";
	myfile << z0 << "\t";
	myfile << t0 << "\t";

	for(int i=0; i < num_of_steps; i++)
	{
	    double  h = (double)rand()/(double)(RAND_MAX/1.0)/10;
	    while(h < 0.001)
    	        h = (double)rand()/(double)(RAND_MAX/1.0)/10;
	    x1 = x0 + (A * sin(z0) + C * cos(y0)) * h;
	    y1 = y0 + (B * sin(x0) + A * cos(z0)) * h;
	    z1 = z0 + (C * sin(y0) + B * cos(x0)) * h;
	    t1 = t0 + h;

	    //put this in the file
	    myfile << x1/1000 + 1.00<< "\t";
	    myfile << y1/1000 + 1.00<< "\t";
	    myfile << z1/1000 + 1.00<< "\t";
	    myfile << t1 << "\t";
	
	    // get rready for next rounds
	    x0 = x1;
	    y0 = y1;
	    z0 = z1;
	    t0 = t1;
	}
	myfile << "\n";


    }
    //close file
    myfile.close();
}

int main()
{
    createABCtracer();
    return 0;
}
