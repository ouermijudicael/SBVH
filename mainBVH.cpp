#include <primitive.h>
#include <SBVHNode.h>
#include <BVHSpliter.h>
#include <KDTREESpliter.h>
#include <tracer.h>

#include <vector>
#include <algorithm>
#include <iostream>
#include <cstdlib>


using namespace std;


int main()
{
     BVHSpliter * s0 = new BVHSpliter();
     //s0->test();
    
     KDTREESpliter * s1 = new KDTREESpliter();
     s1->test();

     cout << "what is going on" << endl;
     return 0;
}
