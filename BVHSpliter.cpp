#include <BVHSpliter.h>
#include <SBVHNode.h>
#include <primitive.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>


#define BVH_MIN_NUM_PRIMITIVES 100 

// for debugging purposes
float BVHdebuger = 0;

using namespace std;

BVHSpliter::BVHSpliter()
{
    tree_size = -10;
    tree.resize(0);
}
BVHSpliter::~BVHSpliter()
{}

// *****************************************
// This function calculates SAH
//******************************************
float BVHSpliter::calculateSAH(float * bbox)
{
    float sah = abs( (bbox[1] - bbox[0]) * (bbox[3] -bbox[2]) * (bbox[7] - bbox[6]) ) +
                 abs( (bbox[1] - bbox[0]) * (bbox[5] -bbox[4]) * (bbox[7] - bbox[6]) ) +
                 abs( (bbox[3] - bbox[2]) * (bbox[5] -bbox[4]) * (bbox[7] - bbox[6]) ) +
                 abs( (bbox[1] - bbox[0]) * (bbox[3] -bbox[2]) * (bbox[5] - bbox[4]) ) ;

	if(sah == 0.00 && abs(bbox[1]-bbox[0]) <= 0.000001)
        {

	     sah = 2*((bbox[3] -bbox[2]) * (bbox[7] - bbox[6]) +
	           (bbox[5] -bbox[4]) * (bbox[7] - bbox[6]) +
	           (bbox[3] -bbox[2]) * (bbox[5] - bbox[4])) ;
	}
	else if(sah ==0 && abs(bbox[3]-bbox[2]) <= 0.000001)
        {
	    sah = 2*( (bbox[1] - bbox[0]) * (bbox[7] - bbox[6]) +
	          (bbox[5] - bbox[4]) * (bbox[7] - bbox[6]) +
	          (bbox[1] - bbox[0]) * (bbox[5] - bbox[4])) ;
	}
	else if(sah ==0 && abs(bbox[5]-bbox[4]) <= 0.000001)
	{

	      sah = 2*( (bbox[1] - bbox[0]) * (bbox[7] - bbox[6]) +
	            (bbox[3] - bbox[2]) * (bbox[7] - bbox[6]) +
	            (bbox[1] - bbox[0]) * (bbox[3] - bbox[2]) ) ;
	}
	else if(sah ==0 && abs(bbox[7]-bbox[6]) <= 0.000001)
	{
	
	    sah = 2 *( (bbox[1] - bbox[0]) * (bbox[3] -bbox[2])  +
	          (bbox[1] - bbox[0]) * (bbox[5] -bbox[4]) +
	          (bbox[3] - bbox[2]) * (bbox[5] -bbox[4]) ) ;
	}

    return sah;
}

//*****************************************************************
// bb1 --> bbox1
// bb2 --> bbox2
// This function takes 2 bbox and updates the first to a new bbox
//*****************************************************************
void BVHSpliter::updateBbox(float * bb1, float * bb2)
{
    if(bb1[0] == -10000 && bb1[1] == -10000)
    {
        for(int i=0; i<4; i++)
        {
	    bb1[i*2] = bb2[i*2];
	    bb1[i*2 + 1] = bb2[i*2 + 1];
    	}
    }
    else
    {
        for(int i=0; i<4; i++)
        {
       	    if(bb2[i*2] < bb1[i*2])
	        bb1[i*2] = bb2[i*2];
	    if(bb2[i*2 + 1] > bb1[i*2 + 1])
	        bb1[i*2 + 1] = bb2[i*2 + 1];
    	}
    }
}

//********************************************************************
// This function appends an id i to a list of object location
//*******************************************************************
void BVHSpliter::append(vector<int>& arr, int id)
{
    arr.push_back(id);
}

void BVHSpliter::append(vector<Segment>& arr, Segment & id)
{
    arr.push_back(id);
}

//************************************************************
// checks if the line crosses the volume of split (4D)
//***********************************************************
bool		BVHSpliter::isOverlaped(float * bb, int dim, float s)
{
    bool result = false;

    if((bb[2*dim+1] - bb[2*dim+0]) >0 && bb[2*dim+0] < s && s<bb[2*dim+1])
	result = true;
    //else if((bb[2*dim+1] - bb[2*dim+0]) <0  && bb[2*dim+1] < s && s < bb[2*dim+1])
	//result = true;
    return result;
}
bool		BVHSpliter::isOverlaped(float *  bb, int dim, float * s)
{
    bool result = false;
    if( (bb[2*dim+1] - bb[2*dim+0]) >0 &&
	((bb[2*dim+0] < s[2*dim+0] && s[2*dim+0]<bb[2*dim+1]) || (bb[2*dim+0] < s[2*dim+1] && s[2*dim+1] < bb[2*dim+1]) ) )
	result = true;
    //else if( (bb[2*dim+1] - bb[2*dim+0]) < 0 &&
    //	((bb[2*dim+1] < s[2*dim+0] && s[2*dim+0]<bb[2*dim+0]) || (bb[2*dim+1] < s[2*dim+1] && s[2*dim+1] < bb[2*dim+0]) ) )
    //	result = true;
    return result;
}

bool		BVHSpliter::isOverlaped(float *  bb, float * s)
{
    bool result = false;
    if( (bb[1] - bb[0]) >0 && (bb[0] <= s[0] && s[0] <= bb[1]) &&
        (bb[3] - bb[2]) >0 && (bb[2] <= s[2] && s[2] <= bb[3]) &&
        (bb[5] - bb[4]) >0 && (bb[4] <= s[4] && s[4] <= bb[5]) &&
        (bb[7] - bb[6]) >0 && (bb[6] <= s[6] && s[6] <= bb[7]) )
	result = true;
    else if( (bb[1] - bb[0]) >0 && (bb[0] <= s[1] && s[1] <= bb[1]) &&
             (bb[3] - bb[2]) >0 && (bb[2] <= s[3] && s[3] <= bb[3]) &&
             (bb[5] - bb[4]) >0 && (bb[4] <= s[5] && s[5] <= bb[5]) &&
             (bb[7] - bb[6]) >0 && (bb[6] <= s[7] && s[7] <= bb[7]) )
	result = true;
    else if( (s[1] - s[0]) >0 && (s[0] <= bb[1] && bb[1] <= s[1]) &&
             (s[3] - s[2]) >0 && (s[2] <= bb[3] && bb[3] <= s[3]) &&
             (s[5] - s[4]) >0 && (s[4] <= bb[5] && bb[5] <= s[5]) &&
             (s[7] - s[6]) >0 && (s[6] <= bb[7] && bb[7] <= s[7]) )
	result = true;
    else if( (s[1] - s[0]) >0 && (s[0] <= bb[1] && bb[1] <= s[1]) &&
             (s[3] - s[2]) >0 && (s[2] <= bb[3] && bb[3] <= s[3]) &&
             (s[5] - s[4]) >0 && (s[4] <= bb[5] && bb[5] <= s[5]) &&
             (s[7] - s[6]) >0 && (s[6] <= bb[7] && bb[7] <= s[7]) )
	result = true;
    
    return result;
}
void BVHSpliter::findBestSpatialSplit(ObjectSplit & result, SBVHNode & node)
{

   //cerr << "Entering findObjectSplit1" << endl;
    int Pl = 0; 
    int Pr = 0;
    int temp_Pl = 0; 
    int temp_Pr = 0;
    float left_sah, right_sah, C;
    int n_node =  node.getNumPrimitives();
    float * bbox = node.getBbox();
    float left_range = 10; 
    float right_range = 10; 

    //float range = 0;
    result.Cost = 1000000;
    result.small_to_split = true;
    result.sort_dim = -10;
    result.Pl = -10;
    result.Pr = -10;
    result.split_location = -10;
    //bool right_bound = true;

    // split_case = 0  --> we consider segements lower bound for the splits
    // split case = 1  --> we consider the segments upper bound for splits 
    int split_case = 0;

    //iset the temp bbox
    float left_bbox[8];
    float  right_bbox[8];
    float temp_split_location = -10000000;
    float * seg_bb;

    //bool left_overlap;
    //bool right_overlap;
    float sah = -10000;
    int step = 1;
    int threshold_num = -10;
    int threshold_num_begin= -10;
    int P_span= 0;
    
    // determine step size to consider
    if(n_node > 1000)
    	step = floor(n_node/1000);
	//step =1;

    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
      split_case = -1;
      while(split_case < 1)
      {// begining of while
        split_case++;

	// sort By axis
    	if(idx_dim == 0 && split_case == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0_lower_bound());
    	else if(idx_dim == 0 && split_case == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0_upper_bound());
    	else if(idx_dim == 0 && split_case == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0());
        else if(idx_dim == 1 && split_case == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1_lower_bound());
    	else if(idx_dim == 1 && split_case == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1_upper_bound());
    	else if(idx_dim == 1 && split_case == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1());
    	else if(idx_dim == 2 && split_case == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2_lower_bound());
    	else if(idx_dim == 2 && split_case == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2_upper_bound());
    	else if(idx_dim == 2 && split_case == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2());
 	else if(idx_dim == 3 && split_case == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3_lower_bound());
    	else if(idx_dim == 3 && split_case == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3_upper_bound());
    	else if(idx_dim == 3 && split_case == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3());
        else 
	{
	   cerr <<  "Not suppose to occur : " << endl;
	}

	//cerr << "range = " << abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) << endl;
	//range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
    	Pl = 0; 
    	Pr = 0;
        temp_Pl =0;
        for(int idx_node=0; idx_node<n_node; idx_node= idx_node +step)
	{
	
	    if(BVHdebuger == 1)
            {
	    //cerr << "idx_node =" << idx_node << endl; 
	    //node.node_spec[idx_node].print();
            }
       	    seg_bb = node.node_spec[idx_node].getBbox();

            if(split_case == 0 && seg_bb[idx_dim*2+0] != bbox[idx_dim*2+0])
            {
            	temp_split_location = seg_bb[idx_dim*2+0];
	    }
	    else if(split_case == 0 && seg_bb[idx_dim*2+0] == bbox[idx_dim*2+0])
            {
		goto label;
            }
	    else if(split_case == 1 && seg_bb[idx_dim*2+1] != bbox[idx_dim*2+1])
	    {
            	temp_split_location = seg_bb[idx_dim*2+1];
            }

	    else if(split_case == 1 && seg_bb[idx_dim*2+1] == bbox[idx_dim*2+1])
	    {
		goto label;
	    }
      	    else if(split_case == 2) // not used yet
	    {
            	temp_split_location = (seg_bb[idx_dim*2+0] + seg_bb[idx_dim*2+1]) /2;
	    }

	    //cerr << "temp split location =" << temp_split_location <<  endl;
	    //cerr << "split case =" << split_case <<  endl;

            // setting temp bounding boxes
            for(int i=0; i<4; i++)
	    {
                if(idx_dim == i)
	        {
		    left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = temp_split_location;
		    right_bbox[i*2+0] = temp_split_location;
		    right_bbox[i*2+1] = bbox[i*2+1];
	        }
		else
		{
	            left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = bbox[i*2+1];
		    right_bbox[i*2+0] = bbox[i*2+0];
		    right_bbox[i*2+1] = bbox[i*2+1];
		}
             }

// Do not remove this tajo
            // set up range to determinw overlaps
	    if((n_node - idx_node) > 10000)
	    	threshold_num = idx_node + 10000;
	    else
	    	threshold_num = n_node;

	    if(idx_node >10000)
		threshold_num_begin =idx_node - 10000;
	    else  
		threshold_num_begin =0;
	    //temp_Pl= 0;
	    temp_Pr = 0;
            P_span = 0;
            for(int idx_tracer= threshold_num_begin; idx_tracer < threshold_num; idx_tracer++)
	    {
		//counting++;
                float * temp_bb = node.node_spec[idx_tracer].getBbox(); 

		if(temp_bb[idx_dim*2+0] < temp_split_location && temp_split_location < temp_bb[idx_dim*2+1])
		     P_span++;
	        //if(temp_split_location>=temp_bb[idx_dim*2+1])
	        //     Pl++;
	    }

            // calculation of number of nodes on each side
            Pl = idx_node + P_span;
	    Pr = n_node - idx_node + P_span;
	    // surface area 
	    left_sah = calculateSAH(left_bbox);
	    right_sah = calculateSAH(right_bbox);
	    sah = node.getSAH();

	    // calculte cost update 
	    C = ((Pl)*left_sah + (Pr)*right_sah) / sah;

            left_range = abs(temp_split_location - bbox[idx_dim*2+0]);
            right_range = abs(temp_split_location - bbox[idx_dim*2+1]);
            //cerr << "Cost saved in result = " << result.Cost << endl; 
	    //cerr << "new Cost = " << C << endl;
            //cerr << "left_sah = " << left_sah << endl;
	    //cerr << "right_sah = "<< right_sah << endl;
	    //cerr << "Pl = " << Pl+temp_Pl << endl;
	    //cerr << "Pr = " << Pr+temp_Pr << endl;
            ///cerr << "sah = " << sah << endl;

	    // update cost
	    if(C < result.Cost && left_range > 0.0001 && right_range > 0.0001)
	     	    //temp_split_location < bbox[idx_dim*2 +1] && temp_split_location > bbox[idx_dim*2+0] ) 
		//&& P_span < n_node/3)
		//&& Pl > floor(n_node /3) && Pr > floor(n_node/3))
	    {
	        //cerr << "dim " << idx_dim << endl;
		//cerr << "Pl = " << Pl << endl;
		//cerr << "Pr = " << Pr << endl;
		//cerr << "Cost =" << C << endl;
		result.Cost = C;
		result.Pl = Pl;
	        result.Pr = Pr;
		result.sort_dim = idx_dim;
		result.split_location = temp_split_location;
		//result.small_to_split = false;
	    }
	    else if(C == result.Cost && left_range > 0.0001 && right_range > 0.0001) 
	     	    //temp_split_location < bbox[idx_dim*2 +1] && temp_split_location > bbox[idx_dim*2+0] ) 
		//&& P_span < n_node/3)
		//&& Pl > floor(n_node /3) && Pr > floor(n_node/3))
	    {
	        //cerr << "dim " << idx_dim << endl;
		//cerr << "Pl = " << Pl << endl;
		//cerr << "Pr = " << Pr << endl;
		//cerr << "Cost =" << C << endl;
		//cerr << "Updating cost" << endl;
                //cerr << " abs(Pl-Pr) = "<<  abs(Pl-Pr) << endl; 
		//cerr << " abs(result.Pr - result.Pl)" << abs(result.Pr - result.Pl) << endl;
                if( abs(Pl-Pr) < abs(result.Pr - result.Pl) )
		{
                   result.Pr = Pr;
		   result.Pl = Pl;
		   result.Cost = C;
		   result.sort_dim = idx_dim;
		   result.split_location = temp_split_location;
		   //result.small_to_split = false;
	  	}
	    }
        //cerr << "Split Cost =" <<result.Cost << endl;
        //cerr << "split locatioin" << result.split_location << endl;
	//cerr << "Pl "<< result.Pl << endl;
	//cerr << "Pr "<< result.Pr << endl;
        //cerr << "sort dim = " << result.sort_dim << endl;
        //cerr << "node bbox =" << bbox[2*result.sort_dim] << "__" << bbox[2*result.sort_dim+1] << endl; 

	label : ;
        }// end of for loop
      }// end of while
      //if(result.split_location == bbox[2*result.sort_dim + 0] || result.split_location == bbox[2*result.sort_dim + 1])
      if(abs(result.split_location- bbox[2*result.sort_dim + 0]) < 0.00001 || abs(result.split_location - bbox[2*result.sort_dim + 1]) < .00001)
      {
        cerr << " Not suppose to occur idx dim = " << idx_dim << endl;
	cerr << "Pl "<< result.Pl << endl;
	cerr << "Pr "<< result.Pr << endl;
        cerr << "Split Cost =" <<result.Cost << endl;
        cerr << "Split location =" <<result.split_location << endl;
        cerr << "Split dim = " << result.sort_dim << endl;;
	cerr << "Split case = " << split_case << endl;
        node.print();
	exit(1);
      }
    }
    //for debuggin purposes 
    // is the bbox values fliped
    if(bbox[result.sort_dim*2+1] < result.split_location || 
       result.split_location < bbox[result.sort_dim*2+0])
    {
	 cerr << "split location" << result.split_location << "is outside of the bounds"<< endl;
 	 cerr << "BNot suppose to occur " << endl;
         cerr << bbox[result.sort_dim*2+1] << "__"<< bbox[result.sort_dim*2+0] << endl;
	 exit(1);
    }
    // end of debugging stuff
    if(result.sort_dim < 0 || result.sort_dim>3)
    {
	cerr << "not suppose to occur" << endl;
	cerr << "Pl "<< result.Pl << endl;
	cerr << "Pr "<< result.Pr << endl;
        cerr << "Split Cost =" <<result.Cost << endl;
        cerr << "Split location =" <<result.split_location << endl;
        cerr << "Split dim = " << result.sort_dim << endl;;
	cerr << "Split case = " << split_case << endl;
        node.print();
	exit(1);
    }

    float range = -10;
    int idx_dim = -10;
    if(result.sort_dim ==-10 || 
	(result.sort_dim == tree[node.getParent()].getSplitDim() &&
	(result.split_location = tree[node.getParent()].getSplitLocation() )) )
    {
        for(int i= 0; i<4; i++)
        {
           if((bbox[2*i+1]-bbox[2*i+0]) >=range && i != result.sort_dim)	
	   {
		range = bbox[2*i+1] - bbox[2*i+0];
		idx_dim = i;
	   }
        }
 
	result.split_location = (bbox[2*idx_dim+0] + bbox[2*idx_dim+1])/2;
        result.sort_dim = idx_dim;
    }
    node.setSplitLocation(result.split_location);
    node.setSplitDim(result.sort_dim);


/***
    int Pl = 0; 
    int Pr = 0;
    int temp_Pl = 0; 
    int temp_Pr = 0;
    float left_sah, right_sah, C;
    int n_node =  node.getNumPrimitives();
    float * bbox = node.getBbox();
    float left_range = 10; 
    float right_range = 10; 

    //float range = 0;
    result.Cost = 1000000;
    result.small_to_split = true;
    result.sort_dim = -10;
    result.Pl = -10;
    result.Pr = -10;
    result.split_location = -10;
    //bool right_bound = true;

    // split_case = 0  --> we consider segements lower bound for the splits
    // split case = 1  --> we consider the segments upper bound for splits 
    int split_case = 0;

    //iset the temp bbox
    float left_bbox[8];
    float  right_bbox[8];
    float temp_split_location = -10000000;
    float * seg_bb;

    //bool left_overlap;
    //bool right_overlap;
    float sah = -10000;
    int step = 1;
    int threshold_num = -10;
    int threshold_num_begin= -10;
    int P_span= 0;
    
    // determine step size to consider
    if(n_node > 1000)
    	step = floor(n_node/1000);
	//step =1;

    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
        // sort By axis
        if(idx_dim == 0)
	       sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0());
        else if(idx_dim == 1)
	       sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1());
        else if(idx_dim == 2)
	       sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2());
        else if(idx_dim == 3)
	       sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3());
        else
        {
	       cerr << "NO WAY TO SPLIT" << endl;
	       exit(1);
        }

    	float split_step = abs(bbox[idx_dim*2+1]-bbox[idx_dim*2+0])/1000;
        for(float idx_split=bbox[idx_dim*2+0]+split_step; idx_split<bbox[idx_dim*2+1]; idx_split += split_step)
        {
	    temp_split_location = idx_split;

 	     // setting temp bounding boxes
            for(int i=0; i<4; i++)
	    {
                if(idx_dim == i)
	        {
		    left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = temp_split_location;
		    right_bbox[i*2+0] = temp_split_location;
		    right_bbox[i*2+1] = bbox[i*2+1];
	        }
		else
		{
	            left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = bbox[i*2+1];
		    right_bbox[i*2+0] = bbox[i*2+0];
		    right_bbox[i*2+1] = bbox[i*2+1];
		}
            }

	    Pl = 0; 
	    Pr = 0;
	    temp_Pl =0;
	    for(int idx_node=0; idx_node<n_node; idx_node++)
	    {
       	    	seg_bb = node.node_spec[idx_node].getBbox();
                float center = node.node_spec[idx_node].getCenter(idx_dim);

	    	// making sure we do not pic a split outiside of bbox
            	if(temp_split_location <= bbox[idx_dim*2+0] || temp_split_location >= bbox[2*idx_dim+1])
	    	{
 		    cerr << "ERROR: OUTSIDE OF BOUNDING BOX" << endl;
		    cerr << "temp split location  tajo=" << temp_split_location <<  endl;
		    cerr << "split step =" << split_step <<  endl;
		    node.print();
		    exit(1);
		}
		//if(temp_split_location >= center)
		if(temp_split_location >= seg_bb[idx_dim*2+0])
	    		Pl++;
                //if(temp_split_location < center)
		if(temp_split_location <= seg_bb[idx_dim*2+1])
			Pr++;
	    }
            //Pl = idx_node;
	    //Pr = n_node - Pl;
	    //Pr = n_node - idx_node + temp_Pr;
	    // surface area 
	    left_sah = calculateSAH(left_bbox);
	    right_sah = calculateSAH(right_bbox);
	    sah = node.getSAH();

	    // calculte cost update 
	    C = ((Pl)*left_sah + (Pr)*right_sah)/sah;

            left_range = abs(temp_split_location - bbox[idx_dim*2+0]);
            right_range = abs(temp_split_location - bbox[idx_dim*2+1]);
	    //cerr << "left_range = " << left_range << endl;
	    //cerr << "right_range = " << right_range << endl;
            //cerr << "Cost saved in result = " << result.Cost << endl; 
	    //cerr << "new Cost = " << C << endl;
            //cerr << "left_sah = " << left_sah << endl;
	    //cerr << "right_sah = "<< right_sah << endl;
	    //cerr << "Pl = " << Pl+temp_Pl << endl;
	    //cerr << "Pr = " << Pr+temp_Pr << endl;
            ///cerr << "sah = " << sah << endl;

	    // update cost
	    if(C < result.Cost && left_range > 0.0001 && right_range > 0.0001 )
		//&& Pl > floor(n_node/ 4))
	    {
	        //cerr << "dim " << idx_dim << endl;
		//cerr << "Pl = " << Pl << endl;
		//cerr << "Pr = " << Pr << endl;
		//cerr << "Cost =" << C << endl;
		result.Cost = C;
		result.Pl = Pl;
	        result.Pr = Pr;
		result.sort_dim = idx_dim;
		result.split_location = temp_split_location;
		//result.small_to_split = false;
	    }
	    else if(C == result.Cost && left_range > 0.0001 && right_range > 0.0001)
		//&& Pl > floor(n_node/ 4))
	    {
	        //cerr << "dim " << idx_dim << endl;
		//cerr << "Pl = " << Pl << endl;
		//cerr << "Pr = " << Pr << endl;
		//cerr << "Cost =" << C << endl;
		//cerr << "Updating cost" << endl;
                //cerr << " abs(Pl-Pr) = "<<  abs(Pl-Pr) << endl; 
		//cerr << " abs(result.Pr - result.Pl)" << abs(result.Pr - result.Pl) << endl;
                if( abs(Pl-Pr) < abs(result.Pr - result.Pl) )
		{
                   result.Pr = Pr;
		   result.Pl = Pl;
		   result.Cost = C;
		   result.sort_dim = idx_dim;
		   result.split_location = temp_split_location;
		   //result.small_to_split = false;
	  	}
	    }
	    //else
	    //{
		//cerr << " Not suppose to be here: I am in the Spatial Split" << endl;
		//exit(1);
            //}
        //cerr << "Split Cost =" <<result.Cost << endl;
        //cerr << "split locatioin" << result.split_location << endl;
	//cerr << "Pl "<< result.Pl << endl;
	//cerr << "Pr "<< result.Pr << endl;
        //cerr << "sort dim = " << result.sort_dim << endl;
        //cerr << "node bbox =" << bbox[2*result.sort_dim] << "__" << bbox[2*result.sort_dim+1] << endl; 

	//label : ;
        }// end of for loop
      //}// end of while
      //if(result.split_location == bbox[2*result.sort_dim + 0] || result.split_location == bbox[2*result.sort_dim + 1])
      if(abs(result.split_location- bbox[2*result.sort_dim + 0]) < 0.00001 || abs(result.split_location - bbox[2*result.sort_dim + 1]) < .00001)
      {
        cerr << " Not suppose to occur idx dim = " << idx_dim << endl;
	cerr << "Pl "<< result.Pl << endl;
	cerr << "Pr "<< result.Pr << endl;
        cerr << "Split Cost =" <<result.Cost << endl;
        cerr << "Split location =" <<result.split_location << endl;
        cerr << "Split dim = " << result.sort_dim << endl;;
	cerr << "Split case = " << split_case << endl;
        node.print();
	exit(1);
      }
    }

    // no good split found just split the date in the middle 
    //float range = bbox[1]-bbox[0];
    float range = -10; //bbox[1]-bbox[0];
    int idx_dim = -10;
    if(result.sort_dim == -10 ||
	(result.sort_dim == tree[node.getParent()].getSplitDim() 
        && result.split_location == tree[node.getParent()].getSplitLocation()) ) // did not find best split just take half
    {
        for(int i= 0; i<4; i++)
        {
           if((bbox[2*i+1]-bbox[2*i+0]) >=range && i != result.sort_dim)	
	   {
		range = bbox[2*i+1] - bbox[2*i+0];
		idx_dim = i;
	   }
        }
       
	result.split_location = (bbox[2*idx_dim+0] + bbox[2*idx_dim+1])/2;
        result.sort_dim = idx_dim;
    }


    // end of debugging stuff
    if(result.sort_dim < 0 || result.sort_dim>3)
    {
	cerr << "not suppose to occur" << endl;
	cerr << "Pl "<< result.Pl << endl;
	cerr << "Pr "<< result.Pr << endl;
        cerr << "Split Cost =" <<result.Cost << endl;
        cerr << "Split location =" <<result.split_location << endl;
        cerr << "Split dim = " << result.sort_dim << endl;;
	cerr << "Split case = " << split_case << endl;
        node.print();
	exit(1);
    }

    node.setSplitLocation(result.split_location);
    node.setSplitDim(result.sort_dim);
*/

/** tajo

    int Pl = 0; 
    int Pr = 0;
    int temp_Pl = 0; 
    int temp_Pr = 0;
    float left_sah, right_sah, C;
    int n_node =  node.getNumPrimitives();
    float * bbox = node.getBbox();
    float left_range = 10; 
    float right_range = 10; 

    //float range = 0;
    result.Cost = 1000000;
    result.small_to_split = true;
    result.sort_dim = -10;
    result.Pl = -10;
    result.Pr = -10;
    result.split_location = -10;
    //bool right_bound = true;

    // split_case = 0  --> we consider segements lower bound for the splits
    // split case = 1  --> we consider the segments upper bound for splits 
    int split_case = 0;

    //iset the temp bbox
    float left_bbox[8];
    float  right_bbox[8];
    float temp_split_location = -10000000;
    float * seg_bb;

    //bool left_overlap;
    //bool right_overlap;
    float sah = -10000;
    int step = 1;
    int threshold_num = -10;
    int threshold_num_begin= -10;
    int P_span= 0;
    
    // determine step size to consider
    if(n_node > 10000)
    	step = floor(n_node/10000);
	//step =1;

    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
       if(idx_dim == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0());
       if(idx_dim == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1());
       if(idx_dim == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2());
       if(idx_dim == 3)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3());


      split_case = -1;
      //while(split_case < 1)
      //{// begining of while
        split_case++;

	//cerr << "range = " << abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) << endl;
	//range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
    	Pl = 0; 
    	Pr = 0;
        temp_Pl =0;
        for(int idx_node=0; idx_node<n_node; idx_node= idx_node +step)
	{
	
	    if(BVHdebuger == 1)
            {
	    //cerr << "idx_node =" << idx_node << endl; 
	    //node.node_spec[idx_node].print();
            }
       	    seg_bb = node.node_spec[idx_node].getBbox();

	    temp_split_location = (seg_bb[idx_dim*2+0] + seg_bb[idx_dim*2+1]) /2;
	    //}

	    //cerr << "temp split location =" << temp_split_location <<  endl;
	    //cerr << "split case =" << split_case <<  endl;

            // setting temp bounding boxes
            for(int i=0; i<4; i++)
	    {
                if(idx_dim == i)
	        {
		    left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = temp_split_location;
		    right_bbox[i*2+0] = temp_split_location;
		    right_bbox[i*2+1] = bbox[i*2+1];
	        }
		else
		{
	            left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = bbox[i*2+1];
		    right_bbox[i*2+0] = bbox[i*2+0];
		    right_bbox[i*2+1] = bbox[i*2+1];
		}
             }

// Do not remove this tajo
            // set up range to determinw overlaps
	    if((n_node - idx_node) > 10000)
	    	threshold_num = idx_node + 10000;
	    else
	    	threshold_num = n_node;

	    if(idx_node >10000)
		threshold_num_begin =idx_node - 10000;
	    else  
		threshold_num_begin =0;
	    //temp_Pl= 0;

            // calculation of number of nodes on each side
            Pl = idx_node + P_span ;
            //Pl = idx_node + temp_Pl;
	    Pr = n_node - idx_node +P_span;
	    //Pr = n_node - idx_node + temp_Pr;
	    // surface area 
	    left_sah = calculateSAH(left_bbox);
	    right_sah = calculateSAH(right_bbox);
	    sah = node.getSAH();

	    // calculte cost update 
	    C = ((Pl)*left_sah + (Pr)*right_sah) / sah;

            left_range = abs(temp_split_location - bbox[idx_dim*2+0]);
            right_range = abs(temp_split_location - bbox[idx_dim*2+1]);
            //cerr << "Cost saved in result = " << result.Cost << endl; 
	    //cerr << "new Cost = " << C << endl;
            //cerr << "left_sah = " << left_sah << endl;
	    //cerr << "right_sah = "<< right_sah << endl;
	    //cerr << "Pl = " << Pl+temp_Pl << endl;
	    //cerr << "Pr = " << Pr+temp_Pr << endl;
            ///cerr << "sah = " << sah << endl;

	    // update cost
	    if(C < result.Cost && left_range > 0.0001 && right_range > 0.0001)// && Pl > floor(n_node /4))
	    {
	        //cerr << "dim " << idx_dim << endl;
		//cerr << "Pl = " << Pl << endl;
		//cerr << "Pr = " << Pr << endl;
		//cerr << "Cost =" << C << endl;
		result.Cost = C;
		result.Pl = Pl;
	        result.Pr = Pr;
		result.sort_dim = idx_dim;
		result.split_location = temp_split_location;
		//result.small_to_split = false;
	    }
	    else if(C == result.Cost && left_range > 0.0001 && right_range > 0.0001)// && Pl > floor(n_node /4))
	    {
	        //cerr << "dim " << idx_dim << endl;
		//cerr << "Pl = " << Pl << endl;
		//cerr << "Pr = " << Pr << endl;
		//cerr << "Cost =" << C << endl;
		//cerr << "Updating cost" << endl;
                //cerr << " abs(Pl-Pr) = "<<  abs(Pl-Pr) << endl; 
		//cerr << " abs(result.Pr - result.Pl)" << abs(result.Pr - result.Pl) << endl;
                if( abs(Pl-Pr) < abs(result.Pr - result.Pl) )
		{
                   result.Pr = Pr;
		   result.Pl = Pl;
		   result.Cost = C;
		   result.sort_dim = idx_dim;
		   result.split_location = temp_split_location;
		   //result.small_to_split = false;
	  	}
	    }
	    //cerr << "Split Cost =" <<result.Cost << endl;
	    //cerr << "split locatioin" << result.split_location << endl;
	    //cerr << "Pl "<< result.Pl << endl;
	    //cerr << "Pr "<< result.Pr << endl;
	    //cerr << "sort dim = " << result.sort_dim << endl;
	    //cerr << "node bbox =" << bbox[2*result.sort_dim] << "__" << bbox[2*result.sort_dim+1] << endl; 

	//label : ;
        }// end of for loop
      //}// end of while
      //if(result.split_location == bbox[2*result.sort_dim + 0] || result.split_location == bbox[2*result.sort_dim + 1])
      if(abs(result.split_location- bbox[2*result.sort_dim + 0]) < 0.00001 || abs(result.split_location - bbox[2*result.sort_dim + 1]) < .00001)
      {
        cerr << " Not suppose to occur idx dim = " << idx_dim << endl;
	cerr << "Pl "<< result.Pl << endl;
	cerr << "Pr "<< result.Pr << endl;
        cerr << "Split Cost =" <<result.Cost << endl;
        cerr << "Split location =" <<result.split_location << endl;
        cerr << "Split dim = " << result.sort_dim << endl;;
	cerr << "Split case = " << split_case << endl;
        node.print();
	exit(1);
      }
    }

    // no good split found just split the date in the middle 
    float range = bbox[1]-bbox[0];
    int idx_dim_temp = -10;
    if(result.sort_dim == -10) // did not find best split just take half
    {
        for(int i= 0; i<4; i++)
        {
           if((bbox[2*i+1]-bbox[2*i+0]) >=range)	
	   {
		range = bbox[2*i+1] - bbox[2*i+0];
		idx_dim_temp = i;
	   }
        }
        int index = floor(n_node/2); 
       
	result.split_location = (bbox[2*idx_dim_temp+0] + bbox[2*idx_dim_temp+1])/2;
        result.sort_dim = idx_dim_temp;
        //cerr << "ENTERED THE CASE " << endl;
        //exit(1);
    }


    //for debuggin purposes 
    // is the bbox values fliped
    if(bbox[result.sort_dim*2+1] < result.split_location || 
       result.split_location < bbox[result.sort_dim*2+0] )
    {
	 cerr << "split location" << result.split_location << "is outside of the bounds"<< endl;
 	 cerr << "BNot suppose to occur " << endl;
         cerr << bbox[result.sort_dim*2+1] << "__"<< bbox[result.sort_dim*2+0] << endl;
	 exit(1);
    }
    // end of debugging stuff
    if(result.sort_dim < 0 || result.sort_dim>3)
    {
	cerr << "not suppose to occur" << endl;
	cerr << "Pl "<< result.Pl << endl;
	cerr << "Pr "<< result.Pr << endl;
        cerr << "Split Cost =" <<result.Cost << endl;
        cerr << "Split location =" <<result.split_location << endl;
        cerr << "Split dim = " << result.sort_dim << endl;;
	cerr << "Split case = " << split_case << endl;
        node.print();
	exit(1);
    }

    node.setSplitLocation(result.split_location);
    node.setSplitDim(result.sort_dim);
tajo**/
}



//**************************************************************
// This use sah to find the best Split 
// for the data
//**************************************************************
void BVHSpliter::findBestObjectSplit(ObjectSplit & result, SBVHNode & node)
{

    int Pl = 0; 
    int Pr = 0;
    int temp_Pl = 0; 
    int temp_Pr = 0;
    float left_sah, right_sah, C;
    int n_node =  node.getNumPrimitives();
    float * bbox = node.getBbox();
    float left_range = 10; 
    float right_range = 10; 

    //float range = 0;
    result.Cost = 1000000;
    result.small_to_split = true;
    result.sort_dim = -10;
    result.Pl = -10;
    result.Pr = -10;
    result.split_location = -10;
    //bool right_bound = true;


    //iset the temp bbox
    float left_bbox[8];
    float  right_bbox[8];
    float temp_split_location = -10000000;
    float * seg_bb;

    //bool left_overlap;
    //bool right_overlap;
    float sah = -10000;
    int step = 1;
    int threshold_num = -10;
    int threshold_num_begin= -10;
    int P_span= 0;
    
    // determine step size to consider
    if(n_node > 1000)
    	step = floor(n_node/1000);
	//step =1;

    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
        if(idx_dim == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0());
        else if(idx_dim == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1());
        else if(idx_dim == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2());
        else if(idx_dim == 3)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3());
        else 
	{
	   cerr <<  "Not suppose to occur : " << endl;
	}

	//range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
    	Pl = 0; 
    	Pr = 0;
        temp_Pl =0;
        for(int idx_node=0; idx_node<n_node; idx_node= idx_node +step)
	{
	
	    if(BVHdebuger == 1)
            {
	    //cerr << "idx_node =" << idx_node << endl; 
	    //node.node_spec[idx_node].print();
            }
       	    seg_bb = node.node_spec[idx_node].getBbox();

	    temp_split_location = (seg_bb[idx_dim*2+0] + seg_bb[idx_dim*2+1]) /2;

	    //cerr << "temp split location =" << temp_split_location <<  endl;

            // setting temp bounding boxes
            for(int i=0; i<4; i++)
	    {
                if(idx_dim == i)
	        {
		    left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = temp_split_location;
		    right_bbox[i*2+0] = temp_split_location;
		    right_bbox[i*2+1] = bbox[i*2+1];
	        }
		else
		{
	            left_bbox[i*2+0] = bbox[i*2+0];
		    left_bbox[i*2+1] = bbox[i*2+1];
		    right_bbox[i*2+0] = bbox[i*2+0];
		    right_bbox[i*2+1] = bbox[i*2+1];
		}
             }

// Do not remove this tajo
            // set up range to determinw overlaps
	    if((n_node - idx_node) > 1000)
	    	threshold_num = idx_node + 1000;
	    else
	    	threshold_num = n_node;

	    if(idx_node >1000)
		threshold_num_begin =idx_node - 1000;
	    else  
		threshold_num_begin =0;
	    //temp_Pl= 0;
            // calculation of number of nodes on each side
            Pl = idx_node;
            //Pl = idx_node + temp_Pl;
	    Pr = n_node - idx_node;
	    //Pr = n_node - idx_node + temp_Pr;
	    // surface area 
	    left_sah = calculateSAH(left_bbox);
	    right_sah = calculateSAH(right_bbox);
	    sah = node.getSAH();

	    // calculte cost update 
	    C = ((Pl)*left_sah + (Pr)*right_sah) / sah;

            left_range = abs(temp_split_location - bbox[idx_dim*2+0]);
            right_range = abs(temp_split_location - bbox[idx_dim*2+1]);
            //cerr << "Cost saved in result = " << result.Cost << endl; 
	    //cerr << "new Cost = " << C << endl;
            //cerr << "left_sah = " << left_sah << endl;
	    //cerr << "right_sah = "<< right_sah << endl;
	    //cerr << "Pl = " << Pl+temp_Pl << endl;
	    //cerr << "Pr = " << Pr+temp_Pr << endl;
            ///cerr << "sah = " << sah << endl;

	    // update cost
	    if(C < result.Cost && left_range > 0.0001 && right_range > 0.0001)// && Pl > floor(n_node /4))
	    {
	        //cerr << "dim " << idx_dim << endl;
		//cerr << "Pl = " << Pl << endl;
		//cerr << "Pr = " << Pr << endl;
		//cerr << "Cost =" << C << endl;
		result.Cost = C;
		result.Pl = Pl;
	        result.Pr = Pr;
		result.sort_dim = idx_dim;
		result.split_location = temp_split_location;
		//result.small_to_split = false;
	    }
	    else if(C == result.Cost && left_range > 0.0001 && right_range > 0.0001)// && Pl > floor(n_node /4))
	    {
	        //cerr << "dim " << idx_dim << endl;
		//cerr << "Pl = " << Pl << endl;
		//cerr << "Pr = " << Pr << endl;
		//cerr << "Cost =" << C << endl;
		//cerr << "Updating cost" << endl;
                //cerr << " abs(Pl-Pr) = "<<  abs(Pl-Pr) << endl; 
		//cerr << " abs(result.Pr - result.Pl)" << abs(result.Pr - result.Pl) << endl;
                if( abs(Pl-Pr) < abs(result.Pr - result.Pl) )
		{
                   result.Pr = Pr;
		   result.Pl = Pl;
		   result.Cost = C;
		   result.sort_dim = idx_dim;
		   result.split_location = temp_split_location;
		   //result.small_to_split = false;
	  	}
	    }
	    //cerr << "Split Cost =" <<result.Cost << endl;
	    //cerr << "split locatioin" << result.split_location << endl;
	    //cerr << "Pl "<< result.Pl << endl;
	    //cerr << "Pr "<< result.Pr << endl;
	    //cerr << "sort dim = " << result.sort_dim << endl;
	    //cerr << "node bbox =" << bbox[2*result.sort_dim] << "__" << bbox[2*result.sort_dim+1] << endl; 

	//label : ;
        //}// end of for loop
      }// end of while
      //if(result.split_location == bbox[2*result.sort_dim + 0] || result.split_location == bbox[2*result.sort_dim + 1])
      if(abs(result.split_location- bbox[2*result.sort_dim + 0]) < 0.00001 || abs(result.split_location - bbox[2*result.sort_dim + 1]) < .00001)
      {
        cerr << " Not suppose to occur idx dim = " << idx_dim << endl;
	cerr << "Pl "<< result.Pl << endl;
	cerr << "Pr "<< result.Pr << endl;
        cerr << "Split Cost =" <<result.Cost << endl;
        cerr << "Split location =" <<result.split_location << endl;
        cerr << "Split dim = " << result.sort_dim << endl;;
        node.print();
	exit(1);
      }
    }
    //for debuggin purposes 
    // is the bbox values fliped
    if(bbox[result.sort_dim*2+1] < result.split_location || 
       result.split_location < bbox[result.sort_dim*2+0] )
    {
	 cerr << "split location" << result.split_location << "is outside of the bounds"<< endl;
 	 cerr << "BNot suppose to occur " << endl;
         cerr << bbox[result.sort_dim*2+1] << "__"<< bbox[result.sort_dim*2+0] << endl;
	 exit(1);
    }
    // end of debugging stuff
    if(result.sort_dim < 0 || result.sort_dim>3)
    {
	cerr << "not suppose to occur" << endl;
	cerr << "Pl "<< result.Pl << endl;
	cerr << "Pr "<< result.Pr << endl;
        cerr << "Split Cost =" <<result.Cost << endl;
        cerr << "Split location =" <<result.split_location << endl;
        cerr << "Split dim = " << result.sort_dim << endl;;
        node.print();
	exit(1);
    }

    node.setSplitLocation(result.split_location);
    node.setSplitDim(result.sort_dim);
}

//**************************************************************
// This use sah to find the best Split 
// for the data
//**************************************************************


//********************************************************
// 
//********************************************************
void BVHSpliter::findBestSplit(ObjectSplit &result, SBVHNode & node)
{
    //cerr << "Entering findObjectSplit1" << endl;

    float sahPl, sahPr, C;
    int n =  node.getNumPrimitives();
    float * bbox = node.getBbox();
    int dim = -1;
    float range = 0;
    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
	if(abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) >= range)
	{
            dim  = idx_dim;
	    range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
	}

        // maybe i need to normalize time
    }
    
/*
    // sort By axis
    if(dim == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0());
    if(dim == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1());
    if(dim == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2());
    if(dim == 3)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3());

*/
    //update cost and slpit location and other elements
    //float center = node.node_spec[n/2].center[dim];
    float center = (bbox[dim*2+1]+bbox[dim*2+0])/2;
    C = -10;
    //if(SBVHdebugSpliter == 1)
    //cerr << "Cost =" << C << endl;
    result.Cost = C;
    result.sort_dim = dim;
    result.split_location = center;


    if(result.sort_dim == -10 || (result.sort_dim == tree[node.getParent()].getSplitDim()  
                  && result.split_location == tree[node.getParent()].getSplitLocation())) // did not find best split just take half
    {

        float range_new = 0.0;
        int dim_new = -1;
        for(int i= 0; i<4; i++)
        {
           if((bbox[2*i+1]-bbox[2*i+0]) >=range_new && i != dim)	
	   {
		range_new = bbox[2*i+1] - bbox[2*i+0];
		dim_new = i;
	   }
        }
       
    	center = (bbox[2*dim_new+1] + bbox[2*dim_new+0]) /2;
	C = -10;
	result.Cost = C;
	result.sort_dim = dim_new;
	result.split_location = center;
	//cerr << "MEDIAN DID NOT WORK FOR INITIAL SPLIT :" <<endl;
	//exit(1);
    }

            
    node.setSplitLocation(result.split_location);
    node.setSplitDim(result.sort_dim);
    //cerr << "Leaving findObjectSplit1" << endl;
}

void BVHSpliter::findObjectSplitMedian(ObjectSplit &result, SBVHNode & node)
{
    //cerr << "Entering findObjectSplit1" << endl;
    int Pl = 0; 
    int Pr = 0;
    float sahPl, sahPr, C;
    int n =  node.getNumPrimitives();
    float * bbox = node.getBbox();
    int dim = -1;
    float range = 0;
    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
	if(abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) >= range)
	{
            dim  = idx_dim;
	    range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
	}

        // maybe i need to normalize time
    }
    
    if(dim == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0());
    if(dim == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1());
    if(dim == 2)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2());
    if(dim == 3)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3());

    int index = floor(n/2); 

    //update cost and slpit location and other elements
    float center = node.node_spec[index].getCenter(dim);
    C = -10;
    result.Cost = C;
    result.sort_dim = dim;
    result.split_location = center;

    node.setSplitLocation(center);
    node.setSplitDim(dim);
            
    //cerr << "Leaving findObjectSplit1" << endl;
}

//************************************************************************
// 
//************************************************************************
void BVHSpliter::performObjectSplit(ObjectSplit & split, SBVHNode & node, SBVHNode & NL, SBVHNode & NR)
{
    //split.print();
    int n =  node.getNumPrimitives();
    int dim = split.sort_dim; 
    float split_pos = split.split_location;
    //int l = split.nP_left;
    //int r = split.nP_right;
    vector<int> l_index;
    vector<Segment> l_node_spec;
    vector<int> r_index;
    vector<Segment> r_node_spec;
    float * bbox = node.getBbox();
    float l_bbox[8];
    float r_bbox[8];

    // initialize bbox
    // this is because i know all my values are positives
    for(int i =0; i<8; i++)
    {
	l_bbox[i] = -10000;
	r_bbox[i] = -10000;
    }

    // end of testing

    int id;
    float * bb2;
    float center;
    for(int i =0; i<n; i++)
    {
        id = node.node_spec[i].id;
        center = node.node_spec[i].getCenter(dim);
        //center = node.node_spec[i].getCenter(dim);
	bb2 = node.node_spec[i].getBbox();
	//bb2 = node.node_spec[i].getBbox();

        /// why it was less than
	//cerr << " i = " << i <<endl;
	//cerr << " id = " << id <<endl;
	//cerr << " node spec id = " << node.node_spec[i].id <<endl;
	//cerr << " data id = " << data[id].id <<endl;
	//node.node_spec[i].print();
	//data[id].print();

	//cerr << "center = " << center << endl;
	//cerr << "split pos =" << split_pos << endl;
        if(center < split_pos)
    	{
  	   append(l_index, id);
  	   append(l_node_spec, node.node_spec[i]);
  	   //append(l_node_spec, node.node_spec[i]);
           updateBbox(l_bbox, bb2);	
	  //testing
	  /*
          float * test_bb = l_node_spec[l_node_spec.size()-1].getBbox();
	  bool test_bool = isOverlaped(l_bbox, test_bb);
	  if(test_bool == false)
          {
	    l_node_spec[l_node_spec.size()-1].print();
	    cerr <<" l_bbox = ";
	    for(int j=0; j<4; j++)
            {
	        cerr << l_bbox[j*2+0] <<"__"<<l_bbox[j*2+1] <<"; ";
	    }
	    cerr <<"****"<<endl;
	  }
	  */
          // edn testing
	}
	else
	{
  	   append(r_index, id);
	   append(r_node_spec, node.node_spec[i]);
           updateBbox(r_bbox, bb2);
	}
    } 
/*
    // testing because i have somenthing weird
    for(int i=0; i<l_node_spec.size(); i++)
    {
        float * test_bb = l_node_spec[i].getBbox();
	bool test_bool = isOverlaped(l_bbox, test_bb);
	if(test_bool == false)
        {
	    l_node_spec[i].print();
	    cerr <<" l_bbox = ";
	    for(int j=0; j<4; j++)
            {
	        cerr << l_bbox[j*2+0] <<"__"<<l_bbox[j*2+1] <<"; ";
	    }
	    cerr <<""<<endl;
	}
    }
    // end testing
*/
    //if(debugSpliter == 1)
        //cerr << "After updating bounding box and ..." << endl; 

    NL.setBbox(l_bbox);
    //NL.setSAH();
    NL.setPrimitivesIndex(l_index, l_index.size());
    NL.setPrimitivesIndex(l_node_spec, l_index.size());
    //NL.setNumPrimitives(l_index.size());

    NR.setBbox(r_bbox);
    //NR.setSAH();
    NR.setPrimitivesIndex(r_index, r_index.size());
    NR.setPrimitivesIndex(r_node_spec, r_index.size());
    //NR.setNumPrimitives(r_index.size());
    
    //if(debugSpliter == 1)
    //{
    	//cerr << "Left child Primitives =" << NL.getNumPrimitives() << endl;
    	//cerr << "Right Child Primitives="<<  NR.getNumPrimitives() << endl;
    	//cerr << "l, r =" << l << ", "<< r << endl;
    	//cerr << "End of perform Split ..." << endl; 
    //}

}

//******************************************************************************************
// 
//******************************************************************************************
void BVHSpliter::buildTree(SBVHNode & node, int me)
{
     //cerr << "In the buildTree" << endl;
     int n = node.getNumPrimitives();
     ObjectSplit split;
     // testing
     /*
     // make sure the ids match
     for(int i=0; i<n; i++)
     {
	int id_test0 = node.getPrimitivesIndex()[i]; 
	int id_test1 = node.node_spec[i].id;
	int id_test2 = data[id_test0].id;
	if(id_test0 != id_test1 || id_test0 != id_test2 || id_test2 != id_test1)
        {
	    cerr << "id  ="<< id_test0 << endl;
	    cerr << "id from node spec =" << id_test1 << endl;
	    cerr << "id  from data=" << id_test2<< endl;
	}
     }
     float * t_bb = node.getBbox();
     for(int i=0; i<n; i++)
     {
      float * bb = node.node_spec[i].getBbox();
      bool test = isOverlaped(t_bb, bb);
     if(test == false)
     {
        node.node_spec[i].print();
	node.print();
     }
     }
     */
     // end testing for debuggin

     SBVHNode nodeL;
     SBVHNode nodeR;

     nodeL.setParent(me);
     nodeR.setParent(me);
     node.setLeftChild(tree.size());
     int l = tree[me].getLeftChild();
     node.setRightChild(tree.size()+1);
     int r = tree[me].getRightChild();

     //cerr << "me =" << node.getMe() << endl;
     //cerr << "l =" << l << " r =" << r << endl;

     //cerr << "In  the middle" << endl;

    // this just allocate memory for the cheildren    
    //node->allocateChildren();
    //cerr << "Debugging line ----" << endl;
    //findObjectSplitMedian(split, tree[me]);
    findBestSpatialSplit(split, tree[me]);
    //findBestSplit(split, tree[me]);
    performObjectSplit(split, tree[me], nodeL, nodeR);
    tree.push_back(nodeL);
    tree.push_back(nodeR);
    int size = tree.size();

    if(BVHdebuger == 1)
    {
        cerr << "---------------------------" << endl;
        cerr << "me " << me << " l =" << l << " r =" << r << endl;
        cerr << "me Primitives =" << tree[me].getNumPrimitives() << endl;
        cerr << "Left child Primitives =" << tree[l].getNumPrimitives() << endl;
        cerr << "Right Child Primitives=" << tree[r].getNumPrimitives() << endl;
        cerr << "split dim =" << tree[me].getSplitDim() << endl;
        cerr << "split locatioin =" << tree[me].getSplitLocation() << endl;
        node.print();
        tree[l].print();
        tree[r].print();
    }

    if(tree[l].getNumPrimitives() > BVH_MIN_NUM_PRIMITIVES && tree[l].isSmall(0.001) != true)
    {
        //cerr << "conditional 1" << endl;
    	buildTree(tree[l], l);
    }
    else
    {
    	tree[l].setLeftChild(-2);
    	tree[l].setRightChild(-2);
    }
    if(tree[r].getNumPrimitives() > BVH_MIN_NUM_PRIMITIVES && tree[r].isSmall(0.001) != true)
    {
        //cerr << "conditional 2" << endl;
    	buildTree(tree[r], r);
    }
    else
    {
    	tree[r].setLeftChild(-2);
    	tree[r].setRightChild(-2);
    }

/*
    if(l==3 || r == 3)
    {
        tree[l].print();
	tree[r].print();
    }
*/
    // clean the shallow copy
    // clean nodes too
    tree[l].clear();
    tree[r].clear();
}

//***********************************************************
//checks if the Node is in the Tree
//************************************************************
bool		BVHSpliter::isInNode(int node_idx, float * pt, float r)
{
    bool result = false;
    //cerr << "checking if it is in the node" << endl;
    //cerr << "number of tracers =" << tree[node_idx].getNumPrimitives() << endl;
    //cerr << "node locatioin =" << node_idx << endl;
    float *  bb = tree[node_idx].getBbox();
    //tree[node_idx].print();

    if( (bb[6] <= pt[3] && pt[3] <= bb[7]) && // checks if time is in interval
        (bb[0] <= pt[0] && pt[0] <= bb[1]) && 
        (bb[2] <= pt[1] && pt[1] <= bb[3]) && 
        (bb[4] <= pt[2] && pt[2] <= bb[5]) )
	result = true; 
	
    else if( (bb[6] <= pt[3] && pt[3] <= bb[7]) && // checks if time is in interval
      ( ((pt[0]-r) <= bb[0] && (pt[0]+r)>= bb[0]) || ((pt[0]-r) <=bb[1] && (pt[0]+r)>=bb[1]) ||
        ((pt[0]-r) >= bb[0] && (pt[0]+r)<= bb[1]) || ((pt[0]-r) <=bb[0] && (pt[0]+r)>=bb[1]) )&&

      ( ((pt[1]-r) <= bb[2] && (pt[1]+r)>= bb[2]) || ((pt[1]-r) <=bb[3] && (pt[1]+r)>=bb[3]) ||
        ((pt[1]-r) >= bb[2] && (pt[1]+r)<= bb[3]) || ((pt[1]-r) <=bb[2] && (pt[1]+r)>=bb[3]) )&&

      ( ((pt[2]-r) <= bb[4] && (pt[2]+r)>= bb[4]) || ((pt[2]-r) <=bb[5] && (pt[2]+r)>=bb[5]) ||
        ((pt[2]-r) >= bb[4] && (pt[2]+r)<= bb[5]) || ((pt[2]-r) <=bb[4] && (pt[2]+r)>=bb[5]) )   )
	result = true;

    return result;
}

void 		BVHSpliter::build()
{
   //SBVHNode * root1 = new SBVHNode();
   SBVHNode root;

   // read file to obtain data
   //read("ABCtest100BASIS");   
   read("ABCtracer");   

/*
	   //if(toy == 1)
	   {
	       Segment s[6];	
	       s[0].pt0[0] = 4.00;    s[3].pt0[0] = 6.00;
	       s[0].pt0[1] = 9.00;    s[3].pt0[1] = 4.00;
	       s[0].pt0[2] = 0.00;    s[3].pt0[2] = 0.00;
	       s[0].pt0[3] = 0.00;    s[3].pt0[3] = 0.00;
	       s[0].pt1[0] = 9.00;    s[3].pt1[0] = 9.00;
	       s[0].pt1[1] = 10.00;   s[3].pt1[1] = 1.00;
	       s[0].pt1[2] = 0.00;    s[3].pt1[2] = 0.00;
	       s[0].pt1[3] = 0.00;    s[3].pt1[3] = 0.00;
	       s[0].setBbox();	      s[3].setBbox();

	       s[1].pt0[0] = 0.00;    s[4].pt0[0] = 1.00;
	       s[1].pt0[1] = 7.00;    s[4].pt0[1] = 3.00;
	       s[1].pt0[2] = 0.00;    s[4].pt0[2] = 0.00;
	       s[1].pt0[3] = 0.00;    s[4].pt0[3] = 0.00;
	       s[1].pt1[0] = 4.00;    s[4].pt1[0] = 7.00;
	       s[1].pt1[1] = 9.00;    s[4].pt1[1] = 2.00;
	       s[1].pt1[2] = 0.00;    s[4].pt1[2] = 0.00;
	       s[1].pt1[3] = 0.00;    s[4].pt1[3] = 0.00;
	       s[1].setBbox();        s[4].setBbox();

	       s[2].pt0[0] = 4.00;    s[5].pt0[0] = 3.00;
	       s[2].pt0[1] = 5.00;    s[5].pt0[1] = 2.00;
	       s[2].pt0[2] = 0.00;    s[5].pt0[2] = 0.00;
	       s[2].pt0[3] = 0.00;    s[5].pt0[3] = 0.00;
	       s[2].pt1[0] = 10.00;   s[5].pt1[0] = 6.00;
	       s[2].pt1[1] = 7.00;    s[5].pt1[1] = 0.00;
	       s[2].pt1[2] = 0.00;    s[5].pt1[2] = 0.00;
	       s[2].pt1[3] = 0.00;    s[5].pt1[3] = 0.00;
	       s[2].setBbox();        s[5].setBbox();
       for(int i=0; i<6; i++)
       {
           s[i].id = i;
           s[i].tracer_id = i;
           data_idx.push_back(data.size());
	   addSegment(s[i]);
	   updateBbox(bbox, s[i].bbox);
       }
   }
///
///
*/
/*
   //if(debugSpliter==1)
   //{
   	//cerr << "done reading file" << endl;
        //print(); 
        //cerr << "beginning of issue" <<endl;
   //}
   // Make sure the indeces are the same
   // for debugging purposes

   for(int i =0; i < data.size(); i++)
   {
	   if(data[i].id != data_idx[i] || data[i].id != i)
	   {
		   cerr << "Not matching index at reading" 
		        << " dtat i =" << i 
		   	<< "data id " << data[i].id 
		   	<< "data index id " << data_idx[i]
		   	<< "data tracer id " << data[i].tracer_id<< endl;
	   }

   }

*/
   root.setPrimitivesIndex(getDataIndex(), getDataSize());
   root.setPrimitivesIndex(data, getDataSize());
   //if(debugSpliter ==1)
   	//cerr << "done seting root primitives indecices" << endl;
   root.setBbox(getDataBbox());
   root.print();
   root.setParent(-1);
   tree.push_back(root); 
   tree_size = 1;


   //delete root

   //cerr << "l child =" << tree[0].getLeftChild() << endl; 
   //tree[0].print();
   //append(root);
   /*
// testing 
    for(int i=0; i<tree[0].getNumPrimitives(); i++)
     {
	int id_test0 = tree[0].getPrimitivesIndex()[i]; 
	int id_test1 = tree[0].node_spec[i].id;
	int id_test2 = data[id_test0].id;
	if(id_test0 != id_test1 || id_test0 != id_test2 || id_test2 != id_test1)
        {
	    cerr << "id  ="<< id_test0 << endl;
	    cerr << "id from node spec =" << id_test1 << endl;
	    cerr << "id  from data=" << id_test2<< endl;
	}
     }
   // end testing
*/

   //tree[0].print();
   buildTree(tree[0], 0);
   tree[0].clear();
}

int traversalBVH = 0;
int final_traversalBVH = 0;
int num_points_BVH = 0;

void 		BVHSpliter::findSmallestNodesWithPoint(int current_node, float *pt, float r, int & ptr_r, int * r_stack)
{
       
	traversalBVH++;
	//cerr <<"ENTERING: traversalBVH = " << traversalBVH << endl; 
// debuging
/*
	int test_node = -1;
        if(current_node == 3)
        {
            test_node = tree[current_node].getRightChild();   
	    int n_node = tree[test_node].getNumPrimitives();
	    vector<int> indeces = tree[test_node].getPrimitivesIndex();
            for(int i=0; i<n_node; i++)
 	    {
		data[indeces[i]].print();
	    }
	}
*/
	int id_left = tree[current_node].getLeftChild();
	int id_right = tree[current_node].getRightChild();
        bool check_left  = false;
        bool check_right = false;
        if(id_left != -2)
	    check_left = isInNode(id_left, pt, r);
	if(id_right != -2)
	    check_right = isInNode(id_right, pt, r);

	//cerr << "pt ( " << pt[0]<<", " << pt[1] <<", " << pt[2]<< ", " << pt[3]<< ")"<< endl;
        //tree[id_left].print();
	//tree[id_right].print();
	//if(check_right == true || check_left == true)
	//{
        //cerr << "The current node is = " << current_node << endl;
	//cerr << "chec_left =" << check_left << endl;
	//cerr << "chec_right =" << check_right << endl;
        //cerr << "id_right = " << id_right << endl;
        //cerr << "id_left  = " << id_left << endl;
	//}
        if(check_left == true)
        {
	    // check if leaf or number of primitives less than threshold
	    if(tree[id_left].getNumPrimitives() <=  BVH_MIN_NUM_PRIMITIVES || 
              (tree[id_left].getRightChild() == -2 && tree[id_left].getLeftChild() == -2) )
            {
	 	ptr_r++;
		r_stack[ptr_r] = id_left;
            }
	    else
	    {
		current_node = tree[current_node].getLeftChild();
	        findSmallestNodesWithPoint(id_left, pt, r, ptr_r, r_stack);
	    }
            
        }
	if(check_right == true)
        {
	    if(tree[id_right].getNumPrimitives() <=  BVH_MIN_NUM_PRIMITIVES || 
	      (tree[id_right].getRightChild() == -2 && tree[id_right].getLeftChild() == -2) )
            {
		//cerr << "should put something in result stack" << endl;
	 	ptr_r++;
		r_stack[ptr_r] = id_right;
            }
	    else
	    {
		current_node = tree[current_node].getRightChild();
	        findSmallestNodesWithPoint(id_right, pt, r, ptr_r, r_stack);
	    }
   	}
	if(check_left == false && check_right == false)
        {
	    ptr_r++;
	    r_stack[ptr_r] = current_node;
	}

	//cerr <<"LEAVING: traversalBVH = " << traversalBVH << endl; 
}


//*************************************************************************
// This function finds the Node that contain the neighbors of pt
// pt[0] = x, pt[1] = y, pt[2] = z, pt[3] = t
//*************************************************************************
void 		BVHSpliter::findNode(float * pt, float r, vector<int> & result)
{
    // check if it is left left
    // check if it is in right child
    int  * result_stack = new int[10000];
    //int path_stack[10000];
    //int ptr_path = -1;
    int ptr_result = -1;
    //ptr_path++;
    //path_stack[ptr_path] = 0;
    int num_primitives;
    int current_node = 0;
    //int id_left;
    //int id_right;
    //bool check_left;
    //bool check_right;
    //int visit;
    //int traversal = 0;
    clock_t find_while_loop_time, find_for_loop_time;


    find_while_loop_time = clock();
    findSmallestNodesWithPoint(current_node, pt, r, ptr_result, result_stack);
    find_while_loop_time = clock() - find_while_loop_time;

    float radius;
    bool test = false;
    vector<int> indeces;
    
    //int wrong_node = 0;
    if(BVHdebuger == 1)
    {
        cerr << "number of node with result = " << ptr_result+1 << endl;
        cerr << "number of visits in the tree = " << traversalBVH << endl;
    }
    
    if(traversalBVH > 1)
    {
	final_traversalBVH = final_traversalBVH + traversalBVH;
	num_points_BVH++;
	//cerr <<"NOT FINISHED" << endl;
	//exit(1);
    }
    traversalBVH = 0;
    find_for_loop_time = clock();

    for(int stack_idx=0; stack_idx<ptr_result+1; stack_idx++)
    {
        
	current_node = result_stack[stack_idx];
        //cerr << "current node = " << current_node << endl;
        // for testing  purposes
	//bool test1 = isInNode(current_node, pt);
        //if(test1 == false)
        //    wrong_node++;	
        // end of testing 
	//current_node = tree[current_node].getParent();
        num_primitives = tree[current_node].getNumPrimitives();

        clock_t indeces_time;
        clock_t node_time;
        if(BVHdebuger == 1)
	{
	//cerr << "after while " << endl;
	//cerr << "num primitives =" << num_primitives << endl;
	//tree[current_node].print();
        //indeces_time = clock();
      
	}
	indeces = tree[current_node].getPrimitivesIndex();
        if(BVHdebuger == 1)
        {
        //indeces_time = clock() - indeces_time;
        //cerr << "indeces of node " << current_node << " time =  "<< indeces_time << endl;
	// check that the resul is in the limits
	// if not it goes pack to the parent 
        
        //node_time = clock();
        }
	for(int i =0; i<num_primitives; i++)
	{
	    int id_result = indeces[i];
            if(id_result == 4)
            {
		//cerr << "tajo testing " << endl;
		//data[id_result].print();
		//cerr << "pt = (" <<pt[0] <<", " << pt[1] <<", " << pt[2] << ", " << pt[3] << ")" << endl;
            }
	    test = data[id_result].findPosition(pt);      	
	    //cerr << "test =" << test << endl;
	    if(test == true)
	    {
                radius = data[id_result].getRadius(pt);
		if(radius <= r)
 	           result.push_back(id_result);
	    }
	}
        
        if(BVHdebuger == 1)
	{
	//node_time = clock() - node_time;
        //cerr << "element " << current_node << " time =  "<< node_time << endl;
	}
    }

    find_for_loop_time = clock() - find_for_loop_time;
    // check for radius and grow
    if(BVHdebuger == 1)
    {
        cerr << "find_while_loop_time = " << find_while_loop_time << endl;
        cerr << "find_for_loop_time = " << find_for_loop_time<< endl;
    }
    // free result and path stack
    delete [] result_stack;
    //delete [] path_stack;
}

//*************************************************************
// This is the traditional find slow for loop 
// fro comparaison purposes
//**************************************************************
void 		BVHSpliter::traditionalFind(float * pt, float r, vector<int> & result)
{
    for(int i=0; i < getDataSize(); i++)
    {
	bool test = data[i].findPosition(pt);      	
	//cerr << "test =" << test << endl;
	if(test == true)
	{
	    float radius = data[i].getRadius(pt);
		//cerr << " radius = " << radius << endl;
		//cerr << "target radius = " << r << endl;
	    if(radius <= r)
		 result.push_back(i);
	}
    }
}
/*
//*************************************************************
// Uses preordertraversal to claculate the size of tree
//*************************************************************
void 		BVHSpliter::getTreeSizeInByte(SBVHNode & root, int & result)
{
   int result_l =0;
   int result_r =0;
   if(root.getLeftChild() !=-2)
	getTreeSizeInByte(tree[root.getLeftChild()], result_l);
   if(root.getRightChild() !=-2)
	getTreeSizeInByte(tree[root.getRightChild()], result_r);
   //cerr << result_l << "------" << result_r << endl;
   result = result + result_l + result_r + sizeof(root) + root.getSizeInByte();
}

*/
//*************************************************************
// testing while I go
//*************************************************************
void 		BVHSpliter::test()
{
    clock_t build_time, search_time;
    float build_time_sec, search_time_sec;
    static float pt[4] = {.59, .59, .59, 4.8};
    static float temp_pt[4] = {2.00, 3.00, 0.00, 0.00};
    vector<int> result_0;
    vector<int> result_01;
    vector<int> result_1;

    cout << "*********************Before Build*********************" << endl;
    build_time = clock();
    build();
    //tree[0].print();
    build_time = clock() - build_time;
    cout << "*********************After Build*********************" << endl;
    build_time_sec =((float)build_time) / CLOCKS_PER_SEC;

    // create point that are unformly distributed in space
    int numb = 10;
    float X[numb];
    float Y[numb];
    float Z[numb];
    float T[numb];
    float * bb_node0 = tree[0].getBbox();


    float deltaX = (bb_node0[1]- bb_node0[0])/numb;
    float deltaY = (bb_node0[3]- bb_node0[2])/numb;
    float deltaZ = (bb_node0[5]- bb_node0[4])/numb;
    float deltaT = (bb_node0[7]- bb_node0[6])/numb;

    X[0] = bb_node0[0];
    Y[0] = bb_node0[2];
    Z[0] = bb_node0[4];
    T[0] = bb_node0[6];
    for(int i=1; i<numb; i++)
    {
	X[i] = X[i-1]+ deltaX;
	Y[i] = Y[i-1]+ deltaY;
	Z[i] = Z[i-1]+ deltaZ;
	T[i] = T[i-1]+ deltaT;
    }

    float point[numb*numb*numb*numb][4];
    int idx = 0;
    for(int i=1; i<numb; i++)
    {
	for(int j=0; j<numb; j++)
	{
	    for(int k=0; k<numb; k++)
	    {
		for(int m=0; m<numb; m++)
		{
		    point[idx][0] = X[i]; 
		    point[idx][1] = Y[j]; 
		    point[idx][2] = Z[k]; 
		    point[idx][3] = T[m]; 
		    idx++;
		}
	    }
	}
    }
// end of creation of unformely distributed point

   
    //get the number of nodes that are empty
    //getEmptyNodes(tree[0]);
    
    //cerr << "after build" << endl;;
    search_time = clock();

    for(int i=0; i<numb*numb*numb*numb; i++)
    {
        findNode(point[i], .05, result_0);
        //findNode(pt, 0.1, result_0);
        //cerr << "traversal = " << traversalBVH << endl;
	//traversalBVH = 0;
        //findNode(temp_pt, 1.00, result_0);
        //traditionalFind(point[i], .05, result_1);
        //cerr << "*************************" << endl;
	//cerr << "size of result 0 = " << result_0.size() << endl;
        //cerr << "size of result 1 = "  << result_1.size() << endl;
        //for(int i=0; i < result_0.size(); i++)
    	//    cerr << result_0[i] << ", ";
        //cerr<< endl;
        //for(int i=0; i < result_1.size(); i++)
    	//    cerr << result_1[i] << ", ";
        //cerr<< endl;

 	//result_0.clear();
 	//result_1.clear();
    }
    search_time = clock() - search_time;

    search_time_sec = ((float)search_time) / CLOCKS_PER_SEC;
    //sort(result_0.begin(), result_0.end());

    for(int i=0; i<numb*numb*numb*numb; i++)
        traditionalFind(point[i], .05, result_1);
        //traditionalFind(pt, .1, result_1);
        //traditionalFind(temp_pt, 1.00, result_1);
    cout << "*********************BVH Results*********************" << endl;
    cout << "Build time = " << build_time_sec << "		Search time =" << search_time_sec << endl;
    cout << "size of result 0 = " << result_0.size() <<endl;
    cout << "size of result 1 = " << result_1.size() <<endl;
    cout << "Tree size = " << tree.size() << endl;

    for(int i=0; i < result_0.size(); i++)
    	cout << result_0[i] << ", ";
    cout<< endl;
    for(int i=0; i < result_1.size(); i++)
    	cout << result_1[i] << ", ";
    cout<< endl;

    int size_of_tree = tree[0].getSizeOfTree(tree);
    int size_of_data = tree[0].getSizeOfData(data); 
    int num_of_primitives = data.size();
    int num_of_nodes = tree.size();
    cout << "size of data in byte = " << size_of_data<< endl;
    cout << "size of tree in byte = " <<size_of_tree<< endl;

    // save in files
    fstream file;
    file.open("BVHfile", fstream::in | fstream::out | fstream::app);
    file <<  num_of_primitives << "\t";
    file <<  size_of_data << "\t";
    file <<  num_of_nodes << "\t";
    file <<  size_of_tree << "\t";
    file <<  build_time_sec << "\t";
    file <<  search_time_sec << "\t";
    file <<  ((float)final_traversalBVH) / ((float)num_points_BVH) << "\n";
    cerr << "num of points = " << num_points_BVH << endl;
    //tree[1].print();
    //tree[2].print();
    file.close();
}
