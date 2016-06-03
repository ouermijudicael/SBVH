#include <primitive.h>
#include <SBVHNode.h>
#include <SBVHSpliter.h>
#include "SKDTREESpliter.h"
#include <tracer.h>

#include <vector>
#include <algorithm>
#include <iostream>
#include <cstdlib>


using namespace std;

int main()
{
     SBVHSpliter * s0 = new SBVHSpliter();
     SKDTREESpliter * s1 = new SKDTREESpliter();
     s0->test();
     //s1->test();
     cout << "end" << endl;
     return 0;
}
