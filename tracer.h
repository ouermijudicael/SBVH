#include <vector>
#include <iomanip>
#include <iostream>
#include <cmath>

using namespace std;


#ifndef TRACER_H
#define TRACER_H

// added by tajo ......

//**********************************************************************
// segtment struct to hold segments tat constitute the data.
//**********************************************************************
struct Segment
{
    // bbox is the same as the 2 points in segment
    float pt0[4];
    float pt1[4];
    float bbox[8];


    // segment id tell us which tracer it belongs to
    int id;
    int tracer_id;

    // have to be called when segment created;
    void  setBbox()
    {
	for(int i=0; i<4; i++)
	{
	  if(pt0[i] <= pt1[i])
	  {
	      bbox[2*i+0] = pt0[i];
	      bbox[2*i+1] = pt1[i];
	  }
	  else
	  {
	      bbox[2*i+0] = pt1[i];
	      bbox[2*i+1] = pt0[i];
	  }
	}
    }

    // returns point 0
    float * getPoint0()
    {
	    return pt0;
    }

    //returns point 1
    float * getPoint1()
    {
	    return pt1;
    }

    // a general purpose interpolation fontion
    float interpolate(float X, float A, float B, float FA, float FB)
    {
        float result;
	if(A<B && (X < A || X > B) )
	    cerr << "interpolation issue 1 X is outside A and B " << endl;
	else if(A>B && (X>A || X <B))
	    cerr << "interpolation issue 2 X is outside of  A and B " << endl;

	else if(abs(A-B) < 0.0000001 )
	    result = FA;
	else
            result = FA + (X-A)/(B-A) * (FB-FA);
	return result;
    }

    // retuns the center of specific segment determin by dim
    float 		getCenter(int dim) const 
    {
	float result = (bbox[dim*2+1] + bbox[dim*2+0])/2;
	return result;
    }

    // returns pointer to segment bounding box
    float * 		getBbox()
    {
	return bbox;
    }
    float 		getBboxLowerBound(int loc) const
    {
	return bbox[loc];
    }

    float 		getBboxUpperBound(int loc) const
    {
	return bbox[loc];
    }
    // split segment into 2 
    void 		splitSegment(float split_pos, Segment & s, int split_dim)
    {
	float pt[4];
	for(int i=0; i<4; i++)
	{
	    pt[i] = interpolate(split_pos,  pt0[split_dim], pt1[split_dim], pt0[i], pt1[i]);
	    //pt[i] = interpolate(split_pos,  bbox[split_dim*2+0], bbox[split_dim*2+1], bbox[i*2+0], bbox[i*2+1]);
	}
	for(int i=0; i<4; i++)
	{
	    s.pt0[i] = pt[i];
	    s.pt1[i] = pt1[i];
	    pt1[i] = pt[i];
	}
	setBbox();
	s.tracer_id = tracer_id;
	s.setBbox();
    }

    void 		splitSegment2(float split_pos, Segment & s1, Segment & s2, int split_dim)
    {
	float pt[4];
	for(int i=0; i<4; i++)
	{
	    pt[i] = interpolate(split_pos,  pt0[split_dim], pt1[split_dim], pt0[i], pt1[i]);
	    //pt[i] = interpolate(split_pos,  bbox[split_dim*2+0], bbox[split_dim*2+1], bbox[i*2+0], bbox[i*2+1]);
	}
	for(int i=0; i<4; i++)
	{
	    s1.pt0[i] = pt[i];
	    s1.pt1[i] = pt1[i];
            s2.pt0[i] = pt0[i];
	    s2.pt1[i] = pt[i];
	}
	s1.tracer_id = tracer_id;
        s1.id = id;
	s1.setBbox();
	s2.tracer_id = tracer_id;
	s2.id = id;
	s2.setBbox();
    }

    // checks if the point is in the segment
    // here we only care about time
    bool 		findPosition(float * pt)
    {
	bool result = false;
	if( (bbox[6]<=pt[3] && pt[3] <=bbox[7]) && (bbox[0]<=pt[0] && pt[0]<=bbox[1]) &&
	    (bbox[2]<=pt[1] && pt[1] <=bbox[3]) && (bbox[4]<=pt[2] && pt[2]<=bbox[5]) )
	{
	    result = true;
	}
	return result;
    }
    
    // gets the radius of the segement
    float 		getRadius(float * pt)
    {
	    float result = -1;
	    float temp_point[4]; 
	    for(int i=0; i<4; i++)
            {
	    	temp_point[i] = interpolate(pt[3],  pt0[3], pt1[3], pt0[i], pt1[i]);
	    }

	    result = sqrt(pow((temp_point[0]-pt[0]), 2) + pow((temp_point[1]-pt[1]), 2) + 
			  pow((temp_point[2]-pt[2]), 2) ); //+ pow((temp_point[3]-pt[3]), 2) );
	    return result;
    }

    // Asignement operator
    Segment & operator=(const Segment & s)
    {
       	for(int i=0; i<4; i++)
	{
	    pt0[i] = s.pt0[i];
	    pt1[i] = s.pt1[i];
	    bbox[i*2+0] = s.bbox[i*2+0];
	    bbox[i*2+1] = s.bbox[i*2+1];
	}
	tracer_id = s.tracer_id;
	id = s.id; 

    }
    // prints sumary of the segment
    void 		print()
    {
    	cerr << "id = " <<id <<"  bbox (" << setprecision(10)<<bbox[0] << "__" <<bbox[1] <<", "<< bbox[2] << "__" <<bbox[3] <<", "
    	 			          << bbox[4] << "__" <<bbox[5] <<", "<< bbox[6] << "__" <<bbox[7] << ")" << endl;

    }
};

//********************************************************************
// fonr sorting purposes
//********************************************************************

struct less_than0
{
    bool operator()(const Segment & s1, const Segment & s2)
    {
	return (s1.getCenter(0) < s2.getCenter(0));
    }
};
struct less_than0_lower_bound
{
    bool operator()(const Segment & s1, const Segment & s2)
    {
	return (s1.getBboxLowerBound(0) < s2.getBboxLowerBound(0));
    }
};
struct less_than0_upper_bound
{
    bool operator()(const Segment & s1, const Segment & s2)
    {
	return (s1.getBboxLowerBound(1) < s2.getBboxLowerBound(1)); 
    }
};

struct less_than1
{
    bool operator()(const Segment & s1, const Segment & s2)
    {
	return (s1.getCenter(1) < s2.getCenter(1));
    }
};
struct less_than1_lower_bound
{
    bool operator()(const Segment & s1, const Segment & s2)
    {
	return (s1.getBboxLowerBound(2) < s2.getBboxLowerBound(2)); 
    }
};
struct less_than1_upper_bound
{
    bool operator()(const Segment & s1, const Segment & s2)
    {
	return (s1.getBboxUpperBound(3) < s2.getBboxUpperBound(3));
    }
};

struct less_than2
{
    bool operator()(const Segment & s1, const Segment & s2)
    {
	return (s1.getCenter(2) < s2.getCenter(2));
    }
};

struct less_than2_lower_bound
{
    bool operator()(const Segment & s1, const Segment & s2)
    {
	return (s1.getBboxLowerBound(4) < s2.getBboxLowerBound(4)); 
    }
};

struct less_than2_upper_bound
{
    bool operator()(const Segment & s1, const Segment & s2)
    {
	return (s1.getBboxUpperBound(5) < s2.getBboxUpperBound(5));
    }
};


struct less_than3
{
    bool operator()(const Segment & s1, const Segment & s2)
    {
	return (s1.getCenter(3) < s2.getCenter(3));
    }
};

struct less_than3_lower_bound
{
    bool operator()(const Segment & s1, const Segment & s2)
    {
	return (s1.getBboxLowerBound(6) < s2.getBboxLowerBound(6)); 
    }
};

struct less_than3_upper_bound
{
    bool operator()(const Segment & s1, const Segment & s2)
    {
	return (s1.getBboxUpperBound(7) < s2.getBboxUpperBound(7));
    }
};

// end of tajo
#endif

