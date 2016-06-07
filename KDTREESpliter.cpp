#include <KDTREESpliter.h>
#include <SBVHNode.h>
#include <primitive.h>
//#include <tracer.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>

#define KDT_MIN_NUM_PRIMITIVES 200
#define KDT_MAX_TREE_SIZE 1000000

// for debugging purposes
int KDTdebuger = 0;
int median = 0;
using namespace std;

KDTREESpliter::KDTREESpliter()
{
    tree_size = -10;
    tree.resize(0);
}

KDTREESpliter::~KDTREESpliter()
{}

// *****************************************
// This function calculates SAH
//******************************************
float KDTREESpliter::calculateSAH(float * bbox)
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
void KDTREESpliter::updateBbox(float * bb1, float * bb2)
{
    if(bb1[0] == -10000 || bb1[1] == -10000)
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
       	    if(bb2[i*2 + 0] < bb1[i*2 + 0])
	        bb1[i*2 + 0] = bb2[i*2 + 0];
	    if(bb2[i*2 + 1] > bb1[i*2 + 1])
	        bb1[i*2 + 1] = bb2[i*2 + 1];
    	}
    }

}

//********************************************************************
// This function appends an id i to a list of object location
//*******************************************************************
void KDTREESpliter::append(vector<int>& arr, int id)
{
    arr.push_back(id);
}

void KDTREESpliter::append(vector<Segment>& arr, Segment & id)
{
    arr.push_back(id);
}

//NOT USED
bool		KDTREESpliter::lessThanMinimumVolume(float * bb)
{
    bool result = false;

    if( (abs(bb[1]-bb[0]) < .002) && (abs(bb[3]-bb[2]) < .002) && (abs(bb[5]-bb[4]) < .002) && (abs(bb[7]-bb[6]) < .002) )
	result = true;

    return result;
}

//************************************************************
// checks if the line crosses the volume of split (4D)
//***********************************************************
bool		KDTREESpliter::isOverlaped(float * bb, int dim, float s)
{
    bool result = false;

    if((bb[2*dim+1] - bb[2*dim+0]) >0 && bb[2*dim+0] < s && s<bb[2*dim+1])
	result = true;
    //else if((bb[2*dim+1] - bb[2*dim+0]) <0  && bb[2*dim+1] < s && s < bb[2*dim+1])
	//result = true;
    return result;
}

bool		KDTREESpliter::isOverlaped(float *  bb, int dim, float * s)
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

bool		KDTREESpliter::isOverlaped(float *  bb, float * s)
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

/*
//*************************************************************
// Uses preordertraversal to claculate the size of tree
//*************************************************************
void 		KDTREESpliter::getTreeSizeInByte(SBVHNode & root, int & result)
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

//**************************************************************
// This use sah to find the best Split 
// for the data
//**************************************************************
//void 	KDTREESpliter::findBestObjectSplit(ObjectSplitTree & result, SBVHNode & node)
void 	KDTREESpliter::findBestSpatialSplit(ObjectSplitTree & result, SBVHNode & node)
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
    if(n_node > 100)
    	step = floor(n_node/100);
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
	
	    if(KDTdebuger == 1)
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
	    if(C < result.Cost && left_range > 0.0001 && right_range > 0.0001) // &&
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
	    else if(C == result.Cost && left_range > 0.0001 && right_range > 0.0001)// && 
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

    float range = -10;
    int idx_dim = -10;
    if(result.sort_dim ==-10 || result.sort_dim ==1000000 ||
	(result.sort_dim == tree[node.getParent()].getSplitDim() &&
	(result.split_location = tree[node.getParent()].getSplitLocation() ))
	|| (temp_split_location < bbox[idx_dim*2 +1] && temp_split_location > bbox[idx_dim*2+0] ) 
	)
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
/**

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

    	float split_step = abs(bbox[idx_dim*2+1]-bbox[idx_dim*2+0])/100;
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
                //float center = node.node_spec[idx_node].getCenter(idx_dim);

	    	// making sure we do not pic a split outiside of bbox
            	if(temp_split_location <= bbox[idx_dim*2+0] || temp_split_location >= bbox[2*idx_dim+1])
	    	{
 		    cerr << "ERROR: OUTSIDE OF BOUNDING BOX" << endl;
		    cerr << "temp split location  tajo=" << temp_split_location <<  endl;
		    cerr << "split step =" << split_step <<  endl;
		    node.print();
		    exit(1);
		}
		if(temp_split_location >= bbox[idx_dim*2+0])
	    		Pl++;
                if(temp_split_location <= bbox[idx_dim*2+1])
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
    float range = bbox[1]-bbox[0];
    int idx_dim = -10;
    if(result.sort_dim == -10) // did not find best split just take half
    {
        for(int i= 0; i<4; i++)
        {
           if((bbox[2*i+1]-bbox[2*i+0]) >=range)	
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
}

//**************************************************************
// Not implemented yet 
// for the data
//**************************************************************
void 	KDTREESpliter::findBestObjectSplit(ObjectSplitTree & result, SBVHNode & node)
//void 	KDTREESpliter::findBestSpatialSplit(ObjectSplitTree & result, SBVHNode & node)
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
/*  
    // sort By axis
    if(idx_dim == 0 && (bbox[idx_dim*2+1]-bbox[idx_dim*2+0])> 0.01)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0());
    else if(idx_dim == 1 &&(bbox[idx_dim*2+1]-bbox[idx_dim*2+0])> 0.01)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1());
    else if(idx_dim == 2 &&(bbox[idx_dim*2+1]-bbox[idx_dim*2+0])> 0.01)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2());
    else if(idx_dim == 3 &&(bbox[idx_dim*2+1]-bbox[idx_dim*2+0])> 0.01)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3());
    else
    {
	exit(1);
	 cerr << "NO WAY TO SPLIT" << endl;
    }
*/
      split_case = -1;
      while(split_case < 1)
      {// begining of while
        split_case++;

	// sort By axis
    	if(idx_dim == 0 && split_case == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0_lower_bound());
    	else if(idx_dim == 0 && split_case == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than0_upper_bound());
        else if(idx_dim == 1 && split_case == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1_lower_bound());
    	else if(idx_dim == 1 && split_case == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than1_upper_bound());
    	else if(idx_dim == 2 && split_case == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2_lower_bound());
    	else if(idx_dim == 2 && split_case == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than2_upper_bound());
 	else if(idx_dim == 3 && split_case == 0)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3_lower_bound());
    	else if(idx_dim == 3 && split_case == 1)
            sort(node.getPrimitivesSpec().begin(), node.getPrimitivesSpec().end(),less_than3_upper_bound());
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

       	    seg_bb = node.node_spec[idx_node].getBbox();
	
	    if(KDTdebuger == 1)
            {
	    //cerr << "idx_node =" << idx_node << endl; 
	    //node.node_spec[idx_node].print();
            }

            if(split_case == 0 && seg_bb[idx_dim*2+0] > bbox[idx_dim*2+0] && seg_bb[idx_dim*2+0]<bbox[idx_dim*2+1])
            {
            	temp_split_location = seg_bb[idx_dim*2+0];
	    }
	    else if(split_case == 0 && seg_bb[idx_dim*2+0] == bbox[idx_dim*2+0])
            {
		goto label;
            }
	    else if(split_case == 1 &&  seg_bb[idx_dim*2+1] > bbox[idx_dim*2+0] && seg_bb[idx_dim*2+1]<bbox[idx_dim*2+1])
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
	    else
            {
		goto label;
            }

	    // making sure we do not pic a split outiside of bbox
            if(temp_split_location <= bbox[idx_dim*2+0] || temp_split_location >= bbox[2*idx_dim+1])
	    {
	        //cerr << "temp split location  tajo=" << temp_split_location <<  endl;
	        //cerr << "split case =" << split_case <<  endl;
		goto label;
	    }

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

/*
            for(int idx_tracer=threshold_num_begin; idx_tracer < idx_node; idx_tracer++)
	    {
                float * temp_bb = node.node_spec[idx_tracer].getBbox(); 
	        if(temp_split_location<=temp_bb[idx_dim*2+0])
	        {	    
	             temp_Pl++;
		}
	    }
	    Pl = idx_node - temp_Pl;

*/	    temp_Pr = 0;
	    temp_Pl =0;
            P_span = 0;

            for(int idx_tracer= threshold_num_begin; idx_tracer < threshold_num; idx_tracer++)
	    {
		//counting++;
                float * temp_bb = node.node_spec[idx_tracer].getBbox(); 
		//float center = node.node_spec[idx_tracer].getCenter(idx_dim);

		if(temp_bb[idx_dim*2+0] < temp_split_location && temp_split_location < temp_bb[idx_dim*2+1])
                {
		     P_span++;
		}
	    }

            // calculation of number of nodes on each side
            Pl = idx_node + P_span;
            //Pl = idx_node + temp_Pl;
	    Pr = n_node + P_span - idx_node;
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
	    if(C < result.Cost && left_range > 0.0001 && right_range > 0.0001 &&
	     	    temp_split_location < bbox[idx_dim*2 +1] && temp_split_location > bbox[idx_dim*2+0]) 
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
	    else if(C == result.Cost && left_range > 0.0001 && right_range > 0.0001 &&
	     	    temp_split_location < bbox[idx_dim*2 +1] && temp_split_location > bbox[idx_dim*2+0]) 
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

/*
    // no good split found just split the date in the middle 
    float range = bbox[1]-bbox[0];
    int idx_dim = -10;
    if(result.sort_dim == -10 || (result.sort_dim == tree[node.getParent()].getSplitDim() )) 
                  //&& result.split_location == tree[node.getParent()].getSplitLocation())) // did not find best split just take half
    {
        for(int i= 0; i<4; i++)
        {
           if((bbox[2*i+1]-bbox[2*i+0]) >=range)	
	   {
		range = bbox[2*i+1] - bbox[2*i+0];
		idx_dim = i;
	   }
        }
       
	result.split_location = (bbox[2*idx_dim+0] + bbox[2*idx_dim+1])/2;
        result.sort_dim = idx_dim;
    }

*/
/*
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
*/
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

}


//********************************************************
// This just picks the mide of the data
//********************************************************
void KDTREESpliter::findBestSplit(ObjectSplitTree &result, SBVHNode & node)
{
    //cerr << "Entering findObjectSplit1" << endl;

    float sahPl, sahPr, C;
    int n =  node.getNumPrimitives();
    float * bbox = node.getBbox();
    int dim = -1;
    float range = 0;

    int p; //= node.getparent();
    float p_split_dim; // = tree[p].getsplitdim();
    int r; // = tree[p].getleftchild();
    int l; // = tree[p].getrightchild();
    bool test_p; //  = false;

    p = node.getParent();
    if(p > 0) // if <0 we are at the root
    {
        p_split_dim = tree[p].getSplitDim();
        r = tree[p].getLeftChild();
        l = tree[p].getRightChild();
        test_p  = false;
    }

    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
	if(abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) >= range)
	{
            dim  = idx_dim;
	    range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
	}
    }
    

/*
    //cerr << "Parent right child = " << tree[r].getNumPrimitives() << endl;
    //cerr << "Parent left child = " << tree[l].getNumPrimitives() << endl;
    if(tree[r].getNumPrimitives() ==0 || tree[l].getNumPrimitives() ==0)
    	test_p = true;
    for(int idx_dim = 0; idx_dim < 4; idx_dim++)
    {
        if(test_p == true)
        {
	    if(abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) >= range && idx_dim != p_split_dim)
	    {
                dim  = idx_dim;
	        range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
	    }
           // maybe i need to normalize time
	}
	else
	{
	
	    if(abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]) >= range)
	    {
                dim  = idx_dim;
	        range = abs(bbox[idx_dim*2 + 1] - bbox[idx_dim*2]);
	    }
	}
    }
*/    


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
    //double center = node.node_spec[n/2].center[dim];
    float center = (bbox[dim*2+1]+bbox[dim*2+0])/2;
    if(range < 0.00000000001)
    {
        cerr << "********range too small************************** = " << center << endl;
        cerr << "dim = " << dim << endl;
        cerr << "range " << bbox[2*dim +0] << "___" << bbox[2*dim+1] << endl;
        cerr << "center = " << center << endl;
        node.print();
	exit(1);
    }
 
    C = -10;
    //if(SBVHdebugSpliter == 1)
    //cerr << "Cost =" << C << endl;
    result.Cost = C;
    result.sort_dim = dim;
    result.split_location = center;

    node.setSplitLocation(center);
    node.setSplitDim(dim);
            
    //cerr << "Leaving findObjectSplit1" << endl;
}

void KDTREESpliter::findObjectSplitMedian(ObjectSplitTree & result, SBVHNode & node)
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
       
    	center = node.node_spec[index].getCenter(dim_new);
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

//************************************************************************
// 
//************************************************************************
void KDTREESpliter::performSplit(ObjectSplitTree & split, SBVHNode & node, SBVHNode & NL, SBVHNode & NR)
{
    //cerr << "Entering perform split" << endl;
    //split.print();
    // find the primitives that need to be slpit and split them
    int n =  node.getNumPrimitives();
    int dim = split.sort_dim; 
    float split_pos = split.split_location;
    vector<int> l_index;
    vector<Segment> l_node_spec;
    vector<int> r_index;
    vector<Segment> r_node_spec;

    float * bbox = node.getBbox();
    float l_bbox[8];
    float r_bbox[8];

    // initialize bbox
    for(int i =0; i<4; i++)
    {
	if(i == dim)
        {
	    l_bbox[i*2+0] = bbox[i*2+0];
	    l_bbox[i*2+1] = split_pos;
	    r_bbox[i*2+0] = split_pos;
	    r_bbox[i*2+1] = bbox[i*2+1];
	}
	else
	{
	    l_bbox[i*2+0] = bbox[i*2+0];
	    l_bbox[i*2+1] = bbox[i*2+1];
	    r_bbox[i*2+0] = bbox[i*2+0];
	    r_bbox[i*2+1] = bbox[i*2+1];
	}
    } 
    //cerr << "split pos =" << split_pos << endl;

    //if(debugKDTREE ==1)
    	//cerr << "I have pasted the initialization" << endl; 
    int id;
    //vector<double> line_center;
    float * bb;
    //bool overlap_left;
    //bool overlap_right;
    //float center;

    for(int i =0; i<n; i++)
    {
        id = node.node_spec[i].id;
        if(id > data.size())
        {
	   cerr << "id = " << id << endl;
           cerr << "ID IS OUT OF BOUND " << endl;
	   exit(1);
	}
        //node.node_spec[i].print();
 	//center = node.node_spec[i].getCenter(dim);
        bb = node.node_spec[i].getBbox();
        //overlap_left = isOverlaped(l_bbox, bb);
        //overlap_right = isOverlaped(r_bbox, bb);
	//bool overlap_main = isOverlaped(bbox, bb);

      if(median == 0)
      {
	if(bb[2*dim+0]< split_pos  && split_pos < bb[2*dim +1])
	{
  	   append(l_index, id);
  	   append(l_node_spec, node.node_spec[i]);
  	   append(r_index, id);
	   append(r_node_spec,node.node_spec[i]);
	}
	else if(split_pos >= bb[2*dim+1])
        {
  	   append(l_index, id);
  	   append(l_node_spec, node.node_spec[i]);
        } 
	else if(split_pos <= bb[2*dim+0])
        {
  	   append(r_index, id);
	   append(r_node_spec, node.node_spec[i]);
	}
	else
        {
	    cerr << "Does not overlap with either right or left :" <<endl;
	    exit(1);
	}
      }
      else
      {
	// create 2 new primtives
	if(bb[2*dim+0]< split_pos  && split_pos < bb[2*dim +1])
	{
           Segment s1, s2;
	   node.node_spec[i].splitSegment2(split_pos, s1, s2, dim);   
  	   append(l_index, id);
  	   append(l_node_spec, s1);
  	   append(r_index, id);
	   append(r_node_spec, s2);
	}
	else if(split_pos >= bb[2*dim+1])
        {
  	   append(l_index, id);
  	   append(l_node_spec, node.node_spec[i]);
        } 
	else if(split_pos <= bb[2*dim+0])
        {
  	   append(r_index, id);
	   append(r_node_spec, node.node_spec[i]);
	}
	else
        {
	    cerr << "Does not overlap with either right or left :" <<endl;
	    exit(1);
	}

      }
 
    }    

    //if(debugKDTREE == 1)
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
    
    //if(debugKDTREE == 1)
    //{
    	//cerr << "Left child Primitives =" << NL.getNumPrimitives() << endl;
    	//cerr << "Right Child Primitives="<<  NR.getNumPrimitives() << endl;
    	//cerr << "End of perform Split ..." << endl; 
    //}
    //cerr << "Leaving perform split" << endl;

}

//******************************************************************************************
// 
//******************************************************************************************
void KDTREESpliter::buildTree(SBVHNode & node, int me)
{
     int  n = node.getNumPrimitives();
     //cerr << "In the buildTree" << endl;
     ObjectSplitTree split;

     SBVHNode nodeL;
     SBVHNode nodeR;


     nodeL.setParent(me);
     nodeR.setParent(me);
     node.setLeftChild(tree.size());
     int l = node.getLeftChild();
     node.setRightChild(tree.size()+1);
     int r = node.getRightChild();

     //cerr << "l =" << l << " r =" << r << endl;
     //cerr << "me =" << node.getMe() << endl;

     //cerr << "In  the middle" << endl;

    // this just allocate memory for the cheildren    
    //findBestSplit(split, tree[me]);
    findBestSpatialSplit(split, tree[me]);
    //findObjectSplitMedian(split, tree[me]);
    performSplit(split, tree[me], nodeL, nodeR);

    tree.push_back(nodeL);
    tree.push_back(nodeR);
    int size = tree.size();

    if(KDTdebuger ==1)
    {
         cerr << "---------------------------" << endl;
	 cerr << "me " << me << " l =" << l << " r =" << r << endl;
	 cerr << "me Primitives =" << tree[me].getNumPrimitives() << endl;
	 cerr << "Left child Primitives =" << tree[l].getNumPrimitives() << endl;
	 cerr << "Right Child Primitives="<< tree[r].getNumPrimitives() << endl;
	 cerr << "split dim =" << tree[me].getSplitDim() << endl;
	 cerr << "split locatioin =" << tree[me].getSplitLocation() << endl;;
	 tree[me].print();
	 tree[l].print();
	 tree[r].print();
    }

    if(tree[l].getNumPrimitives() > KDT_MIN_NUM_PRIMITIVES && tree[l].isSmall(0.001) != true 
 	&& tree.size() < KDT_MAX_TREE_SIZE ) 
	//&& tree[l].getNumPrimitives() < tree[me].getNumPrimitives())
    {
        //cerr << "conditional 1" << endl;
    	buildTree(tree[l], l);
    }
    else
    {
        tree[l].setLeftChild(-2);
        tree[l].setRightChild(-2);
    }
    if(tree[r].getNumPrimitives() > KDT_MIN_NUM_PRIMITIVES && tree[r].isSmall(0.001) != true
 	&& tree.size() < KDT_MAX_TREE_SIZE ) 
//	&& tree[r].getNumPrimitives() < tree[me].getNumPrimitives())
    {
        //cerr << "conditional 2" << endl;
    	buildTree(tree[r], r);
    }
    else
    {
        tree[r].setLeftChild(-2);
        tree[r].setRightChild(-2);
    }

    //if(l==3 || r == 3)
    //{
    //    tree[l].print();
    //    tree[r].print();
    //}

    //cerr << "leaving Build tree" << endl;
    // clean the shallow copy
    // clean nodes too
    tree[l].clear();
    tree[r].clear();
}

//***********************************************************
//checks if the Node is in the Tree
//************************************************************
bool		KDTREESpliter::isInNode(int node_idx, float * pt, float r)
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

void 		KDTREESpliter::build()
{
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
   root.setPrimitivesIndex(getDataIndex(), getDataSize());
   root.setPrimitivesIndex(data, getDataSize());
   //if(debugKDTREE ==1)
   	//cerr << "done seting root primitives indecices" << endl;
   root.setBbox(getDataBbox());
   //root.setSAH();
   //if(debugKDTREE ==1)
   	//cerr << "done seting root parent to NULL" << endl;
   
   root.setParent(-1);
   tree.push_back(root);
   tree_size = 1;
   //cerr << "l child =" << tree[0].getLeftChild() << endl; 
   tree[0].print();
   //append(root);
   cerr << "**********************Before Build************" << endl;
   buildTree(tree[0], 0);
   cerr << "*******************After Build***************" << endl;
   tree[0].clear();
   //tree[3].print();
   //if(debugKDTREE ==1)
   	//cerr << "done seting building Tree" << endl;
}
int traversalKDT = 0;
int final_traversalKDT = 0;
int num_points_KDT = 0;
void 		KDTREESpliter::findSmallestNodesWithPoint(int current_node, float *pt, float r, int & ptr_r, int * r_stack)
{
	traversalKDT++;
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
        //cerr << "The current node is = " << current_node << endl;
	//cerr << "chec_left =" << check_left << endl;
	//cerr << "chec_right =" << check_right << endl;
        //cerr << "id_right = " << id_right << endl;
        if(check_left == true)
        {
	    // check if leaf or number of primitives less than threshold
	    if(tree[id_left].getNumPrimitives() <=  KDT_MIN_NUM_PRIMITIVES || 
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
	    if(tree[id_right].getNumPrimitives() <=  KDT_MIN_NUM_PRIMITIVES || 
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
}


//*************************************************************************
// This function finds the Node that contain the neighbors of pt
// pt[0] = x, pt[1] = y, pt[2] = z, pt[3] = t
//*************************************************************************
void 		KDTREESpliter::findNode(float * pt, float r, vector<int> & result)
{
    int * result_stack = new int[100000];
    //int path_stack[10000];
    //int ptr_path = -1;
    int ptr_result = -1;
    int n = tree[0].getNumPrimitives();
    //ptr_path++;
    //path_stack[ptr_path] = 0;

    // root 
    //int dim = tree[0].getSplitDim();
    int num_primitives; //= tree[0].getNumPrimitives();
    int current_node = 0;
    //float split_location = tree[0].getSplitLocation();
    //int id_left;
    //int id_right;
    //bool check_left;
    //bool check_right;
    //int visit;
    //cerr << "Split location = " << split_location << endl; 

    clock_t find_while_loop_time, find_for_loop_time;
    
    find_while_loop_time = clock() ;
    findSmallestNodesWithPoint(current_node, pt, r, ptr_result, result_stack);
    find_while_loop_time = clock() - find_while_loop_time;

    if(traversalKDT>1)
    {
	final_traversalKDT = final_traversalKDT + traversalKDT;
	num_points_KDT++;
    }
    traversalKDT = 0;

    //cerr << "after while " << endl;
    int i = 0;
    vector<int> indeces;
    bool test;      	
    float radius;
    vector<int>::iterator it;
    find_for_loop_time = clock();
    for(int stack_idx=0; stack_idx<ptr_result+1; stack_idx++)
    {
	current_node = result_stack[stack_idx];
        num_primitives = tree[current_node].getNumPrimitives();

	//tree[current_node].print();

	// check that the resul is in the limits
	// if not it goes pack to the parent 
  	//cerr << "**********************current node :" << current_node << endl;
  	//cerr << "tree size :" << tree.size()<< endl;
	//tree[current_node].print();
        
	indeces = tree[current_node].getPrimitivesIndex();
	//cerr << "num primitives =" << num_primitives << endl;
	for(int i =0; i<num_primitives; i++)
	{
	    int id_result = indeces[i];
            //cerr << "id_result = " << id_result << endl;
	    test = data[id_result].findPosition(pt);      	
	    //cerr << "test =" << test << endl;
	    if(test == true)
	    {
                radius = data[id_result].getRadius(pt);
		it = find(result.begin(), result.end(), id_result);
		if(radius <= r && it == result.end())
 	           result.push_back(id_result);
	    }
	}
	//cerr << "num primitives end=" << num_primitives << endl;
    }
    find_for_loop_time = clock() - find_for_loop_time;
    //cerr << "find_while_loop_time = " << find_while_loop_time << endl;
    //cerr << "find_for_loop_time = " << find_for_loop_time << endl;
    // check for radius and grow
    

    delete [] result_stack;
}

//*************************************************************
// This is the traditional find slow for loop 

// fro comparaison purposes
//**************************************************************
void 		KDTREESpliter::traditionalFind(float * pt, float r, vector<int> & result)
{
    for(int i=0; i < getDataSize(); i++)
    {
	bool test = data[i].findPosition(pt);      	
	//cerr << "test =" << test << endl;
	if(test == true)
	{
	    float radius = data[i].getRadius(pt);
	    if(radius <= r)
		 result.push_back(i);
	}
    }
}

//*************************************************************
// testing while I go
//*************************************************************
void 		KDTREESpliter::test()
{
    clock_t build_time, search_time;
    float build_time_sec, search_time_sec;
    //static float pt[4] = {1.00, 1.00, 1.00, 4.8};
    static float pt[4] = {.99, .99, .99, 4.5};
    static float temp_pt[4] = {2.00, 3.00, 0.00, 0.00};
    vector<int> result_0;
    vector<int> result_01;
    vector<int> result_1;

    build_time = clock();
    build();
    //tree[0].print();
    build_time = clock() - build_time;
    build_time_sec =((float)build_time) / CLOCKS_PER_SEC;
/*
//testing
    for(int k=0; k<tree.size(); k++)
    {
        tree[k].print();
	vector<int> index = tree[k].getPrimitivesIndex();
    }
    exit(1);
*/
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

    X[0] = 0.0;
    Y[0] = 0.0;
    Z[0] = 0.0;
    T[0] = 0.0;
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
        findNode(point[i], .05, result_0);
        //findNode(temp_pt, 1.00, result_0);

    search_time = clock() - search_time;
    search_time_sec = ((float)search_time) / CLOCKS_PER_SEC;
    //sort(result_0.begin(), result_0.end());

    for(int i=0; i<numb*numb*numb*numb; i++)
        traditionalFind(point[i], .05, result_1);
        //traditionalFind(temp_pt, 1.00, result_1);

    cout << "*********************KDTREE Results*********************" << endl;
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
    cout << "size of tree in byte = " << size_of_tree<< endl;
    cout << "Number of points  = " << num_points_KDT<< endl;


    // save in files
    fstream file;
    file.open("KDTfile", fstream::in | fstream::out | fstream::app);
    file <<  num_of_primitives << "\t";
    file <<  size_of_data << "\t";
    file <<  num_of_nodes << "\t";
    file <<  size_of_tree << "\t";
    file <<  build_time_sec << "\t";
    file <<  search_time_sec << "\t";
    file <<  ((float)final_traversalKDT)/((float)num_points_KDT) << "\n";
    file.close();
}
