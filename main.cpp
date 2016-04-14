#include <primitive.h>
#include <SBVHNode.h>
//#include <SBVHSpliter.h>
#include <BVHSpliter.h>
//#include <SKDTREESpliter.h>
#include <KDTREESpliter.h>
#include <tracer.h>

#include <vector>
#include <algorithm>
#include <iostream>
#include <cstdlib>


using namespace std;

int main()
{
     //SBVHData * s = new SBVHData();
     //s->test();
     //SBVHNode * node = new SBVHNode();
     //node->test();

     //SBVHSpliter * split = new SBVHSpliter();
     //split->test();

     //SKDTREESpliter * skdt_split = new SKDTREESpliter();
     //skdt_split->test();

     //BVHSpliter * bvh_split = new BVHSpliter();
     //bvh_split->test();

     KDTREESpliter * kdt_split = new KDTREESpliter();
     kdt_split->test();

     cout << "what is going on" << endl;
     return 0;
}
